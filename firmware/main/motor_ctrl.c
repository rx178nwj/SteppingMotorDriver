#include "motor_ctrl.h"
#include "gpio_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "motor_ctrl";

/* ------------------------------------------------------------------ */
/*  RMT 設定定数                                                        */
/* ------------------------------------------------------------------ */
#define RMT_RESOLUTION_HZ   1000000UL
#define STEP_HIGH_TICKS     2U
#define STEP_LOW_MIN_TICKS  2U
/* バッチを小さくして速度更新の応答性を確保する */
#define RMT_LOOP_COUNT      5U

/* ------------------------------------------------------------------ */
/*  モーションデフォルト                                                 */
/* ------------------------------------------------------------------ */
#define DEFAULT_VMAX    1000UL   /* steps/sec */
#define DEFAULT_ACCEL    500UL   /* steps/sec² */
#define DEFAULT_DECEL    500UL   /* steps/sec² */
/* 起動初速: 最初のRMTバッチを短時間で終わらせるための最低速度 */
#define START_VEL_MIN    50.0f   /* steps/sec */
#define START_VEL_MAX   200.0f

/* ------------------------------------------------------------------ */
/*  内部軸状態                                                           */
/* ------------------------------------------------------------------ */
static const int s_step_gpios[NUM_AXES] = STEP_GPIOS;
static const int s_dir_gpios[NUM_AXES]  = DIR_GPIOS;

typedef struct {
    /* RMT */
    rmt_channel_handle_t tx_chan;
    rmt_encoder_handle_t copy_enc;
    rmt_symbol_word_t    sym;
    volatile bool        running;

    /* 状態機械 */
    axis_state_t         state;
    bool                 dir;           /* true = 正方向 */

    /* 位置 (on_trans_done でバッチ単位に更新) */
    volatile int32_t     step_pos;

    /* モーション目標 */
    int32_t              target_pos;
    bool                 pos_mode;      /* true = MOVE/MOVETO, false = VEL */

    /* 速度プロファイル [steps/sec] */
    float                current_vel;
    float                target_vel;    /* VEL モード用 */
    uint32_t             v_max;
    uint32_t             accel;
    uint32_t             decel;

    /* ソフトリミット */
    int32_t              min_pos;
    int32_t              max_pos;
} axis_t;

static axis_t s_axes[NUM_AXES];
static bool   s_drv_enabled = false;

/* ------------------------------------------------------------------ */
/*  ヘルパー: sym の周波数更新                                           */
/* ------------------------------------------------------------------ */
static void set_rmt_freq(axis_t *ax, float freq_hz)
{
    if (freq_hz < 1.0f)        freq_hz = 1.0f;
    if (freq_hz > 200000.0f)   freq_hz = 200000.0f;

    uint32_t period = (uint32_t)((float)RMT_RESOLUTION_HZ / freq_hz);
    uint32_t low    = (period > STEP_HIGH_TICKS) ? (period - STEP_HIGH_TICKS)
                                                  : STEP_LOW_MIN_TICKS;
    if (low < STEP_LOW_MIN_TICKS) low = STEP_LOW_MIN_TICKS;
    /* duration フィールドは 15bit = 最大 32767 ticks (≈30 Hz 以上をサポート) */
    if (low > 32767U)             low = 32767U;

    ax->sym.duration0 = (uint16_t)STEP_HIGH_TICKS;
    ax->sym.duration1 = (uint16_t)low;
}

/* ------------------------------------------------------------------ */
/*  ヘルパー: RMT バッチ送信                                             */
/* ------------------------------------------------------------------ */
static esp_err_t rmt_kick(axis_t *ax)
{
    const rmt_transmit_config_t cfg = {
        .loop_count      = RMT_LOOP_COUNT,
        .flags.eot_level = 0,
    };
    return rmt_transmit(ax->tx_chan, ax->copy_enc,
                        &ax->sym, sizeof(rmt_symbol_word_t), &cfg);
}

/* ------------------------------------------------------------------ */
/*  on_trans_done コールバック — ステップ計数のみ（ISR コンテキスト）    */
/*  rmt_transmit() は ISR 非安全のため呼ばない。                        */
/*  再キューは motor_control_task がタスクコンテキストから行う。         */
/* ------------------------------------------------------------------ */
static bool IRAM_ATTR on_trans_done(rmt_channel_handle_t tx_chan,
                                    const rmt_tx_done_event_data_t *edata,
                                    void *user_ctx)
{
    axis_t *ax = (axis_t *)user_ctx;
    if (!ax->running) return false;

    /* バッチ分のステップを位置に反映 */
    if (ax->dir) {
        ax->step_pos += (int32_t)RMT_LOOP_COUNT;
    } else {
        ax->step_pos -= (int32_t)RMT_LOOP_COUNT;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/*  MotorControlTask — 1ms ティック台形プロファイルエンジン              */
/* ------------------------------------------------------------------ */
static void motor_control_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));

        for (int i = 0; i < NUM_AXES; i++) {
            axis_t *ax = &s_axes[i];

            if (ax->state != AXIS_ACCEL &&
                ax->state != AXIS_CRUISE &&
                ax->state != AXIS_DECEL) {
                continue;
            }

            /* 残りステップ (位置モード) */
            int32_t remaining = 0;
            if (ax->pos_mode) {
                remaining = ax->dir
                    ? (ax->target_pos - ax->step_pos)
                    : (ax->step_pos  - ax->target_pos);
            }

            /* 現在速度で停止するのに必要なステップ数 */
            float decel_steps = (ax->current_vel * ax->current_vel) /
                                 (2.0f * (float)ax->decel);

            switch (ax->state) {
            case AXIS_ACCEL: {
                ax->current_vel += (float)ax->accel * 0.001f;
                /* VEL モード: target_vel と v_max の小さい方で頭打ち */
                float vel_cap = ax->pos_mode
                    ? (float)ax->v_max
                    : (ax->target_vel < (float)ax->v_max ? ax->target_vel : (float)ax->v_max);
                if (ax->current_vel >= vel_cap) {
                    ax->current_vel = vel_cap;
                    ax->state = AXIS_CRUISE;
                }
                if (ax->pos_mode && remaining <= (int32_t)decel_steps) {
                    ax->state = AXIS_DECEL;
                }
                set_rmt_freq(ax, ax->current_vel);
                break;
            }

            case AXIS_CRUISE:
                if (!ax->pos_mode) {
                    /* VEL モード: 目標速度に向けて加減速 */
                    float tv = ax->target_vel;
                    if (ax->current_vel < tv) {
                        ax->current_vel += (float)ax->accel * 0.001f;
                        if (ax->current_vel > tv) ax->current_vel = tv;
                        set_rmt_freq(ax, ax->current_vel);
                    } else if (ax->current_vel > tv) {
                        ax->current_vel -= (float)ax->decel * 0.001f;
                        if (ax->current_vel < tv) ax->current_vel = tv;
                        set_rmt_freq(ax, ax->current_vel);
                    }
                } else {
                    if (remaining <= (int32_t)decel_steps) {
                        ax->state = AXIS_DECEL;
                    }
                }
                break;

            case AXIS_DECEL:
                ax->current_vel -= (float)ax->decel * 0.001f;
                if (ax->current_vel < 1.0f ||
                    (ax->pos_mode && remaining <= 0)) {
                    ax->current_vel = 0.0f;
                    ax->running = false;
                    ax->state   = AXIS_IDLE;
                    ESP_LOGI(TAG, "Axis %d: done, pos=%ld", i, (long)ax->step_pos);
                } else {
                    set_rmt_freq(ax, ax->current_vel);
                }
                break;

            default:
                break;
            }

            /* タスクコンテキストから次のバッチを再キュー。
             * trans_queue_depth=4 なので満杯時は ESP_ERR_INVALID_STATE を返す — 無視して良い。 */
            if (ax->running) {
                rmt_kick(ax);
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  motor_ctrl_init                                                     */
/* ------------------------------------------------------------------ */
void motor_ctrl_init(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        axis_t *ax = &s_axes[i];

        const rmt_tx_channel_config_t ch_cfg = {
            .gpio_num          = s_step_gpios[i],
            .clk_src           = RMT_CLK_SRC_DEFAULT,
            .resolution_hz     = RMT_RESOLUTION_HZ,
            .mem_block_symbols = 48,
            .trans_queue_depth = 4,
            .flags.invert_out  = false,
            .flags.with_dma    = false,
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&ch_cfg, &ax->tx_chan));

        const rmt_copy_encoder_config_t enc_cfg = {};
        ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &ax->copy_enc));

        const rmt_tx_event_callbacks_t cbs = { .on_trans_done = on_trans_done };
        ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(ax->tx_chan, &cbs, ax));

        ax->sym.level0 = 1;
        ax->sym.level1 = 0;
        set_rmt_freq(ax, DEFAULT_VMAX);

        ESP_ERROR_CHECK(rmt_enable(ax->tx_chan));

        ax->running     = false;
        ax->state       = AXIS_IDLE;
        ax->dir         = true;
        ax->step_pos    = 0;
        ax->target_pos  = 0;
        ax->pos_mode    = true;
        ax->current_vel = 0.0f;
        ax->target_vel  = 0.0f;
        ax->v_max       = DEFAULT_VMAX;
        ax->accel       = DEFAULT_ACCEL;
        ax->decel       = DEFAULT_DECEL;
        ax->min_pos     = INT32_MIN / 2;
        ax->max_pos     = INT32_MAX / 2;

        ESP_LOGI(TAG, "Axis %d RMT init OK (GPIO %d)", i, s_step_gpios[i]);
    }

    xTaskCreate(motor_control_task, "MotorCtrl", 4096, NULL, 20, NULL);
    ESP_LOGI(TAG, "MotorControlTask started");
}

/* ------------------------------------------------------------------ */
/*  motor_enable / motor_disable                                        */
/* ------------------------------------------------------------------ */
void motor_enable(void)
{
    gpio_set_level(GPIO_DRV_EN, 0);   /* アクティブ Low */
    s_drv_enabled = true;
    ESP_LOGI(TAG, "DRV enabled");
}

void motor_disable(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        axis_t *ax = &s_axes[i];
        ax->running     = false;
        ax->current_vel = 0.0f;
        ax->state       = AXIS_IDLE;
        rmt_disable(ax->tx_chan);
        rmt_enable(ax->tx_chan);
        gpio_set_level(s_step_gpios[i], 0);
    }
    gpio_set_level(GPIO_DRV_EN, 1);
    s_drv_enabled = false;
    ESP_LOGI(TAG, "DRV disabled");
}

/* ------------------------------------------------------------------ */
/*  start_motion — 内部ヘルパー                                         */
/* ------------------------------------------------------------------ */
static bool start_motion(uint8_t axis, bool dir, int32_t target, bool pos_mode)
{
    if (axis >= NUM_AXES) return false;
    if (!s_drv_enabled) {
        ESP_LOGW(TAG, "Axis %d: motion rejected — call ENABLE first", axis);
        return false;
    }
    axis_t *ax = &s_axes[axis];
    if (ax->state == AXIS_FAULT) return false;

    /* 既存モーション停止 */
    if (ax->running) {
        ax->running = false;
        rmt_tx_wait_all_done(ax->tx_chan, pdMS_TO_TICKS(200));
        rmt_disable(ax->tx_chan);
        rmt_enable(ax->tx_chan);
    }

    /* 位置モード: ソフトリミット & 同一位置チェック */
    if (pos_mode) {
        if (target > ax->max_pos) target = ax->max_pos;
        if (target < ax->min_pos) target = ax->min_pos;
        if (target == ax->step_pos) return true;
    }

    /* DIR 設定 (STEP 開始前に 650ns 以上確保 — 1ms delay で十分) */
    ax->dir = dir;
    gpio_set_level(s_dir_gpios[axis], dir ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    ax->target_pos  = target;
    ax->pos_mode    = pos_mode;

    /* 初速: 最初のバッチ (RMT_LOOP_COUNT パルス) を短時間で完了させる最低速度 */
    float sv = (float)ax->v_max * 0.1f;
    if (sv < START_VEL_MIN) sv = START_VEL_MIN;
    if (sv > START_VEL_MAX) sv = START_VEL_MAX;
    ax->current_vel = sv;
    ax->state       = AXIS_ACCEL;

    set_rmt_freq(ax, ax->current_vel);
    ax->running = true;

    esp_err_t err = rmt_kick(ax);
    if (err != ESP_OK) {
        ax->running = false;
        ax->state   = AXIS_IDLE;
        ESP_LOGE(TAG, "Axis %d rmt_kick: %s", axis, esp_err_to_name(err));
        return false;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/*  motor_move / motor_moveto                                           */
/* ------------------------------------------------------------------ */
bool motor_move(uint8_t axis, int32_t steps)
{
    if (axis >= NUM_AXES || steps == 0) return (steps == 0);
    int32_t target = s_axes[axis].step_pos + steps;
    return start_motion(axis, steps > 0, target, true);
}

bool motor_moveto(uint8_t axis, int32_t pos)
{
    if (axis >= NUM_AXES) return false;
    int32_t steps = pos - s_axes[axis].step_pos;
    if (steps == 0) return true;
    return start_motion(axis, steps > 0, pos, true);
}

/* ------------------------------------------------------------------ */
/*  motor_vel — 速度モード                                              */
/* ------------------------------------------------------------------ */
bool motor_vel(uint8_t axis, int32_t vel_signed)
{
    if (axis >= NUM_AXES) return false;
    if (vel_signed == 0) return motor_stop(axis);

    axis_t *ax = &s_axes[axis];
    bool    dir   = vel_signed > 0;
    float   speed = (float)(vel_signed < 0 ? -vel_signed : vel_signed);
    if (speed > 200000.0f) speed = 200000.0f;

    if (ax->running && ax->state != AXIS_FAULT) {
        /* 既存 VEL モード: 目標速度だけ更新 (方向変更は再起動) */
        if (ax->dir == dir) {
            ax->target_vel = speed;
            if (speed > (float)ax->v_max) ax->target_vel = (float)ax->v_max;
            ax->state = AXIS_CRUISE;
            return true;
        }
        /* 方向変更: 一旦停止してから再起動 */
        ax->running = false;
        rmt_tx_wait_all_done(ax->tx_chan, pdMS_TO_TICKS(200));
        rmt_disable(ax->tx_chan);
        rmt_enable(ax->tx_chan);
    }

    ax->target_vel = speed;
    /* VEL モードでは target_pos は使わない */
    bool ok = start_motion(axis, dir, 0, false);
    if (ok) {
        /* start_motion は ACCEL で開始するが v_max を上書きしないため
         * target_vel に向けて CRUISE フェーズで到達する */
        ax->target_vel = speed;
    }
    return ok;
}

/* ------------------------------------------------------------------ */
/*  motor_stop / motor_estop                                           */
/* ------------------------------------------------------------------ */
bool motor_stop(uint8_t axis)
{
    if (axis >= NUM_AXES) return false;
    axis_t *ax = &s_axes[axis];
    if (!ax->running) return true;

    ax->pos_mode = false;   /* 位置目標を解除して DECEL → IDLE */
    ax->state    = AXIS_DECEL;
    ESP_LOGI(TAG, "Axis %d: STOP (decel)", axis);
    return true;
}

void motor_estop(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        axis_t *ax = &s_axes[i];
        ax->running     = false;
        ax->current_vel = 0.0f;
        ax->state       = AXIS_FAULT;
        rmt_disable(ax->tx_chan);
        rmt_enable(ax->tx_chan);
        gpio_set_level(s_step_gpios[i], 0);
    }
    ESP_LOGE(TAG, "ESTOP: all axes faulted");
}

/* ------------------------------------------------------------------ */
/*  motor_clear_fault                                                  */
/* ------------------------------------------------------------------ */
void motor_clear_fault(void)
{
    /* DRV8825 リセットパルス (最小 10µs、ここでは 1ms) */
    gpio_set_level(GPIO_DRV_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(GPIO_DRV_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    for (int i = 0; i < NUM_AXES; i++) {
        if (s_axes[i].state == AXIS_FAULT) {
            s_axes[i].state = AXIS_IDLE;
        }
    }
    ESP_LOGI(TAG, "FAULT cleared");
}

/* ------------------------------------------------------------------ */
/*  motor_get_status                                                   */
/* ------------------------------------------------------------------ */
bool motor_get_status(uint8_t axis, axis_status_t *out)
{
    if (axis >= NUM_AXES || !out) return false;
    axis_t *ax = &s_axes[axis];
    out->state = ax->state;
    out->pos   = ax->step_pos;
    out->vel   = ax->dir
        ?  (int32_t)ax->current_vel
        : -(int32_t)ax->current_vel;
    out->v_max = ax->v_max;
    out->accel = ax->accel;
    out->decel = ax->decel;
    return true;
}

/* ------------------------------------------------------------------ */
/*  パラメータ設定                                                       */
/* ------------------------------------------------------------------ */
bool motor_set_vmax(uint8_t axis, uint32_t vmax)
{
    if (axis >= NUM_AXES || vmax == 0 || vmax > 200000UL) return false;
    s_axes[axis].v_max = vmax;
    return true;
}

bool motor_set_accel(uint8_t axis, uint32_t accel_val)
{
    if (axis >= NUM_AXES || accel_val == 0) return false;
    s_axes[axis].accel = accel_val;
    return true;
}

bool motor_set_decel(uint8_t axis, uint32_t decel_val)
{
    if (axis >= NUM_AXES || decel_val == 0) return false;
    s_axes[axis].decel = decel_val;
    return true;
}

/* ================================================================== */
/*  Phase 1 テスト API — デバッグ用 (以下変更なし)                      */
/* ================================================================== */

bool motor_test_pulse(uint8_t axis, uint32_t freq_hz)
{
    if (axis >= NUM_AXES) return false;
    if (freq_hz == 0 || freq_hz > 200000UL) return false;

    axis_t *ax = &s_axes[axis];
    uint32_t period_ticks = RMT_RESOLUTION_HZ / freq_hz;
    uint32_t low_ticks    = period_ticks - STEP_HIGH_TICKS;
    if (low_ticks < STEP_LOW_MIN_TICKS) low_ticks = STEP_LOW_MIN_TICKS;
    if (low_ticks > 32767U)             low_ticks = 32767U;

    if (ax->running) {
        ax->running = false;
        rmt_tx_wait_all_done(ax->tx_chan, pdMS_TO_TICKS(500));
        rmt_disable(ax->tx_chan);
        rmt_enable(ax->tx_chan);
    }

    ax->sym.level0    = 1;
    ax->sym.duration0 = (uint16_t)STEP_HIGH_TICKS;
    ax->sym.level1    = 0;
    ax->sym.duration1 = (uint16_t)low_ticks;
    ax->running       = true;

    const rmt_transmit_config_t tx_cfg = {
        .loop_count      = RMT_LOOP_COUNT,
        .flags.eot_level = 0,
    };
    esp_err_t err = rmt_transmit(ax->tx_chan, ax->copy_enc,
                                 &ax->sym, sizeof(rmt_symbol_word_t), &tx_cfg);
    if (err != ESP_OK) {
        ax->running = false;
        ESP_LOGE(TAG, "rmt_transmit axis%d: %s", axis, esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Axis %d: %lu Hz started", axis, (unsigned long)freq_hz);
    return true;
}

bool motor_stop_immediate(uint8_t axis)
{
    if (axis >= NUM_AXES) return false;
    axis_t *ax = &s_axes[axis];

    ax->running     = false;
    ax->current_vel = 0.0f;
    ax->state       = AXIS_IDLE;
    rmt_disable(ax->tx_chan);
    rmt_enable(ax->tx_chan);
    gpio_set_level(s_step_gpios[axis], 0);

    ESP_LOGI(TAG, "Axis %d: stopped", axis);
    return true;
}

bool motor_test_gpio_toggle(uint8_t axis, uint32_t count)
{
    if (axis >= NUM_AXES) return false;
    axis_t *ax = &s_axes[axis];

    ax->running = false;
    rmt_disable(ax->tx_chan);

    gpio_reset_pin(s_step_gpios[axis]);
    gpio_set_direction(s_step_gpios[axis], GPIO_MODE_OUTPUT);

    ESP_LOGI(TAG, "Axis %d GPIO%d: toggling %lu times at ~1Hz",
             axis, s_step_gpios[axis], (unsigned long)count);

    for (uint32_t i = 0; i < count; i++) {
        gpio_set_level(s_step_gpios[axis], 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(s_step_gpios[axis], 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    rmt_enable(ax->tx_chan);
    ESP_LOGI(TAG, "Axis %d: RMT re-enabled", axis);
    return true;
}
