#include "motor_ctrl.h"
#include "gpio_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/portmacro.h"

static const char *TAG = "motor_ctrl";

/* ------------------------------------------------------------------ */
/*  RMT 設定定数                                                        */
/* ------------------------------------------------------------------ */
#define RMT_RESOLUTION_HZ   1000000UL   /* 1 MHz → 1 µs/tick */
#define STEP_HIGH_TICKS     2U           /* 2 µs HIGH (DRV8825 最小 1.9 µs) */
#define STEP_LOW_MIN_TICKS  2U           /* 2 µs LOW  (DRV8825 最小 1.9 µs) */
#define RMT_LOOP_COUNT      5U           /* バッチサイズ: 速度更新応答性確保 */

/* ------------------------------------------------------------------ */
/*  モーションデフォルト (config.c の DEF_* と合わせる)                  */
/* ------------------------------------------------------------------ */
#define DEFAULT_VMAX    10000UL   /* steps/sec  */
#define DEFAULT_ACCEL   50000UL   /* steps/sec² */
#define DEFAULT_DECEL   50000UL   /* steps/sec² */
#define DEFAULT_IDLE_TIMEOUT_MS  2000U

/* 起動初速: 最初の RMT バッチを短時間で完了させる最低速度 */
#define START_VEL_MIN    50.0f
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
    bool                 disable_on_stop; /* true = STOP_FREE: 停止後コイル解除 */

    /* 速度プロファイル [steps/sec] */
    float                current_vel;
    float                target_vel;    /* VEL モード用 */
    uint32_t             v_max;
    uint32_t             accel;
    uint32_t             decel;

    /* ソフトリミット */
    int32_t              min_pos;
    int32_t              max_pos;

    /* アイドルタイムアウト */
    uint32_t             idle_counter_ms;
} axis_t;

static axis_t   s_axes[NUM_AXES];
static bool     s_drv_enabled = false;
static uint32_t s_idle_timeout_ms = DEFAULT_IDLE_TIMEOUT_MS;

/* step_pos の ISR ↔ タスク競合を防ぐスピンロック (F-MOT-05 / 9.8) */
static portMUX_TYPE s_step_pos_mux = portMUX_INITIALIZER_UNLOCKED;

/* フォルト情報（最後の FAULT を記録） */
static fault_info_t     s_last_fault   = { FAULT_NONE, 0, 0 };

/* イベントコールバック */
static motor_event_cb_t  s_move_done_cb = NULL;
static motor_fault_cb_t  s_fault_cb     = NULL;

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
/*  ヘルパー: RMT 停止（ISR 非安全 → タスクコンテキストからのみ呼ぶ）     */
/* ------------------------------------------------------------------ */
static void rmt_stop_channel(axis_t *ax)
{
    ax->running = false;
    rmt_disable(ax->tx_chan);
    rmt_enable(ax->tx_chan);
}

/* ------------------------------------------------------------------ */
/*  on_trans_done コールバック（ISR コンテキスト）                        */
/*  rmt_transmit() は ISR 非安全のため呼ばない。                         */
/*  再キューは motor_control_task がタスクコンテキストから行う。           */
/* ------------------------------------------------------------------ */
static bool IRAM_ATTR on_trans_done(rmt_channel_handle_t tx_chan,
                                    const rmt_tx_done_event_data_t *edata,
                                    void *user_ctx)
{
    axis_t *ax = (axis_t *)user_ctx;
    if (!ax->running) return false;

    /* バッチ分のステップを位置に反映 (F-MOT-05: ISR↔タスク競合防止) */
    taskENTER_CRITICAL_ISR(&s_step_pos_mux);
    if (ax->dir) {
        ax->step_pos += (int32_t)RMT_LOOP_COUNT;
    } else {
        ax->step_pos -= (int32_t)RMT_LOOP_COUNT;
    }
    taskEXIT_CRITICAL_ISR(&s_step_pos_mux);
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

        /* --- アイドルタイムアウト監視 --- */
        bool any_active = false;
        for (int i = 0; i < NUM_AXES; i++) {
            axis_state_t st = s_axes[i].state;
            if (st == AXIS_ACCEL || st == AXIS_CRUISE ||
                st == AXIS_DECEL || st == AXIS_HOMING) {
                any_active = true;
                break;
            }
        }

        for (int i = 0; i < NUM_AXES; i++) {
            axis_t *ax = &s_axes[i];

            /* --- アイドルカウンタ --- */
            if (ax->state == AXIS_IDLE) {
                ax->idle_counter_ms++;
                if (s_idle_timeout_ms > 0 &&
                    ax->idle_counter_ms >= s_idle_timeout_ms &&
                    !any_active) {
                    /* 全軸アイドル到達 → スリープ遷移（全軸共通信号は1回だけ操作） */
                    if (s_drv_enabled) {
                        gpio_set_level(GPIO_DRV_EN, 1);     /* コイル解除 */
                        gpio_set_level(GPIO_DRV_SLEEP, 0);  /* スリープ   */
                        s_drv_enabled = false;
                    }
                    ax->state = AXIS_SLEEP;
                    ax->idle_counter_ms = 0;
                    ESP_LOGI(TAG, "Axis %d: idle timeout → SLEEP", i);
                }
            } else if (ax->state != AXIS_SLEEP && ax->state != AXIS_FAULT) {
                ax->idle_counter_ms = 0;
            }

            /* --- 速度プロファイル更新 --- */
            if (ax->state != AXIS_ACCEL &&
                ax->state != AXIS_CRUISE &&
                ax->state != AXIS_DECEL) {
                continue;
            }

            /* 残りステップ (位置モード) — step_pos は ISR と共有 */
            int32_t remaining = 0;
            if (ax->pos_mode) {
                taskENTER_CRITICAL(&s_step_pos_mux);
                int32_t pos = ax->step_pos;
                taskEXIT_CRITICAL(&s_step_pos_mux);
                remaining = ax->dir
                    ? (ax->target_pos - pos)
                    : (pos - ax->target_pos);
            }

            /* 現在速度で停止するのに必要なステップ数 */
            float decel_steps = (ax->current_vel * ax->current_vel) /
                                 (2.0f * (float)ax->decel);

            switch (ax->state) {
            case AXIS_ACCEL: {
                ax->current_vel += (float)ax->accel * 0.001f;
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
                    ax->idle_counter_ms = 0;

                    /* STOP_FREE: コイル解除 */
                    if (ax->disable_on_stop) {
                        ax->disable_on_stop = false;
                        gpio_set_level(GPIO_DRV_EN, 1);
                        s_drv_enabled = false;
                    }

                    ESP_LOGI(TAG, "Axis %d: done, pos=%ld", i, (long)ax->step_pos);

                    /* EVT MOVE_DONE コールバック（位置モードのみ） */
                    if (ax->pos_mode && s_move_done_cb) {
                        s_move_done_cb((uint8_t)i);
                    }
                } else {
                    set_rmt_freq(ax, ax->current_vel);
                }
                break;

            default:
                break;
            }

            /* タスクコンテキストから次のバッチを再キュー。
             * trans_queue_depth=4 なので満杯時は ESP_ERR_INVALID_STATE を返す — 無視してよい。 */
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

        ax->running         = false;
        ax->state           = AXIS_IDLE;
        ax->dir             = true;
        ax->step_pos        = 0;
        ax->target_pos      = 0;
        ax->pos_mode        = true;
        ax->disable_on_stop = false;
        ax->current_vel     = 0.0f;
        ax->target_vel      = 0.0f;
        ax->v_max           = DEFAULT_VMAX;
        ax->accel           = DEFAULT_ACCEL;
        ax->decel           = DEFAULT_DECEL;
        ax->min_pos         = -2000000;
        ax->max_pos         =  2000000;
        ax->idle_counter_ms = 0;

        ESP_LOGI(TAG, "Axis %d RMT init OK (GPIO %d)", i, s_step_gpios[i]);
    }

    xTaskCreate(motor_control_task, "MotorCtrl", 4096, NULL, 20, NULL);
    ESP_LOGI(TAG, "MotorControlTask started");
}

/* ------------------------------------------------------------------ */
/*  コールバック登録                                                      */
/* ------------------------------------------------------------------ */
void motor_register_move_done_cb(motor_event_cb_t cb) { s_move_done_cb = cb; }
void motor_register_fault_cb(motor_fault_cb_t cb)     { s_fault_cb = cb; }

/* ------------------------------------------------------------------ */
/*  motor_enable / motor_disable                                        */
/* ------------------------------------------------------------------ */
void motor_enable(void)
{
    gpio_set_level(GPIO_DRV_SLEEP, 1);   /* スリープ解除 */
    vTaskDelay(pdMS_TO_TICKS(1));         /* チャージポンプ安定待ち */
    gpio_set_level(GPIO_DRV_EN, 0);       /* アクティブ Low: 励磁 */
    s_drv_enabled = true;

    /* SLEEP 状態の軸を IDLE に戻す */
    for (int i = 0; i < NUM_AXES; i++) {
        if (s_axes[i].state == AXIS_SLEEP) {
            s_axes[i].state = AXIS_IDLE;
            s_axes[i].idle_counter_ms = 0;
        }
    }
    ESP_LOGI(TAG, "DRV enabled");
}

void motor_disable(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        axis_t *ax = &s_axes[i];
        ax->running     = false;
        ax->current_vel = 0.0f;
        ax->state       = AXIS_IDLE;
        rmt_stop_channel(ax);
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

    /* 位置モード: ソフトリミット & 同一位置チェック */
    if (pos_mode) {
        if (target > ax->max_pos) target = ax->max_pos;
        if (target < ax->min_pos) target = ax->min_pos;
        taskENTER_CRITICAL(&s_step_pos_mux);
        int32_t cur_pos = ax->step_pos;
        taskEXIT_CRITICAL(&s_step_pos_mux);
        if (target == cur_pos) return true;
    }

    /* DIR 設定 (STEP 開始前に 650ns 以上確保 — 1ms delay で十分) */
    ax->dir = dir;
    gpio_set_level(s_dir_gpios[axis], dir ? 1 : 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    ax->target_pos      = target;
    ax->pos_mode        = pos_mode;
    ax->disable_on_stop = false;
    ax->idle_counter_ms = 0;

    /* 初速: 最初のバッチを短時間で完了させる最低速度 */
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
    /* E008: モーション実行中は拒否 */
    if (motor_is_moving(axis)) return false;
    taskENTER_CRITICAL(&s_step_pos_mux);
    int32_t cur = s_axes[axis].step_pos;
    taskEXIT_CRITICAL(&s_step_pos_mux);
    int32_t target = cur + steps;
    return start_motion(axis, steps > 0, target, true);
}

bool motor_moveto(uint8_t axis, int32_t pos)
{
    if (axis >= NUM_AXES) return false;
    /* E008: モーション実行中は拒否 */
    if (motor_is_moving(axis)) return false;
    taskENTER_CRITICAL(&s_step_pos_mux);
    int32_t cur = s_axes[axis].step_pos;
    taskEXIT_CRITICAL(&s_step_pos_mux);
    int32_t steps = pos - cur;
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
        /* 既存 VEL モード（同方向）: 目標速度だけ更新 */
        if (!ax->pos_mode && ax->dir == dir) {
            ax->target_vel = speed;
            if (speed > (float)ax->v_max) ax->target_vel = (float)ax->v_max;
            ax->state = AXIS_CRUISE;
            return true;
        }
        /* MOVE/MOVETO 実行中: 減速停止後に VEL モードへ移行 (VEL 割り込みシーケンス) */
        ax->pos_mode   = false;
        ax->target_vel = speed;
        ax->state      = AXIS_DECEL;
        /* DECEL 完了後に再起動は行わない — 現実装では IDLE になる。
         * VEL 割り込みシーケンスの完全実装は Phase 2 で対応。 */
        return true;
    }

    ax->target_vel = speed;
    bool ok = start_motion(axis, dir, 0, false);
    if (ok) {
        ax->target_vel = speed;
    }
    return ok;
}

/* ------------------------------------------------------------------ */
/*  motor_stop / motor_stop_free / motor_estop                         */
/* ------------------------------------------------------------------ */
bool motor_stop(uint8_t axis)
{
    if (axis >= NUM_AXES) return false;
    axis_t *ax = &s_axes[axis];
    if (!ax->running) return true;

    ax->pos_mode        = false;
    ax->disable_on_stop = false;
    ax->state           = AXIS_DECEL;
    ESP_LOGI(TAG, "Axis %d: STOP (decel)", axis);
    return true;
}

bool motor_stop_free(uint8_t axis)
{
    if (axis >= NUM_AXES) return false;
    axis_t *ax = &s_axes[axis];

    if (!ax->running) {
        /* 既に停止中: 即時コイル解除 */
        gpio_set_level(GPIO_DRV_EN, 1);
        s_drv_enabled = false;
        return true;
    }

    ax->pos_mode        = false;
    ax->disable_on_stop = true;   /* DECEL 完了後にコイル解除 */
    ax->state           = AXIS_DECEL;
    ESP_LOGI(TAG, "Axis %d: STOP_FREE (decel then disable)", axis);
    return true;
}

void motor_estop(fault_reason_t reason)
{
    uint8_t mask = 0;
    for (int i = 0; i < NUM_AXES; i++) {
        axis_t *ax = &s_axes[i];
        if (ax->state != AXIS_SLEEP) {
            mask |= (1u << i);
        }
        ax->running     = false;
        ax->current_vel = 0.0f;
        ax->state       = AXIS_FAULT;
        rmt_stop_channel(ax);
        gpio_set_level(s_step_gpios[i], 0);
    }

    /* DRV_EN → High（コイル励磁解除）: FAULT 遷移処理 手順2 */
    gpio_set_level(GPIO_DRV_EN, 1);
    s_drv_enabled = false;

    /* フォルト情報記録 */
    s_last_fault.reason       = reason;
    s_last_fault.axis_mask    = mask;
    s_last_fault.timestamp_us = esp_timer_get_time();

    ESP_LOGE(TAG, "ESTOP: reason=%d mask=0x%02x", (int)reason, mask);

    /* EVT FAULT コールバック */
    if (s_fault_cb) {
        s_fault_cb(reason, mask);
    }
}

/* ------------------------------------------------------------------ */
/*  motor_clear_fault  (F-MOT-07b)                                     */
/* ------------------------------------------------------------------ */
bool motor_clear_fault(void)
{
    /* 全軸が FAULT 状態であることを確認 */
    bool any_fault = false;
    for (int i = 0; i < NUM_AXES; i++) {
        if (s_axes[i].state == AXIS_FAULT) {
            any_fault = true;
            break;
        }
    }
    if (!any_fault) return false;   /* ERR E010: 非 FAULT 状態 */

    /* 1. DRV_EN → High（念のため確認） */
    gpio_set_level(GPIO_DRV_EN, 1);
    s_drv_enabled = false;

    /* 2-3. DRV_RESET パルス（最低 10 µs、ここでは 1 ms） */
    gpio_set_level(GPIO_DRV_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(GPIO_DRV_RESET, 1);

    /* 4. 内部初期化完了待ち */
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 5-6. DRV_SLEEP → High（動作可能状態）＋チャージポンプ安定待ち */
    gpio_set_level(GPIO_DRV_SLEEP, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 7. DRV_EN は High のまま維持（励磁は ENABLE コマンドで別途行う） */

    /* 8. 全軸状態を SLEEP に遷移 */
    for (int i = 0; i < NUM_AXES; i++) {
        if (s_axes[i].state == AXIS_FAULT) {
            s_axes[i].state = AXIS_SLEEP;
            s_axes[i].idle_counter_ms = 0;
        }
    }

    ESP_LOGI(TAG, "FAULT cleared → SLEEP");
    return true;
}

/* ------------------------------------------------------------------ */
/*  motor_get_fault_info  (F-MOT-07c)                                  */
/* ------------------------------------------------------------------ */
void motor_get_fault_info(fault_info_t *out)
{
    if (out) *out = s_last_fault;
}

/* ------------------------------------------------------------------ */
/*  motor_get_status / motor_is_moving                                 */
/* ------------------------------------------------------------------ */
bool motor_get_status(uint8_t axis, axis_status_t *out)
{
    if (axis >= NUM_AXES || !out) return false;
    axis_t *ax = &s_axes[axis];
    out->state = ax->state;
    taskENTER_CRITICAL(&s_step_pos_mux);
    out->pos = ax->step_pos;
    taskEXIT_CRITICAL(&s_step_pos_mux);
    out->vel   = ax->dir
        ?  (int32_t)ax->current_vel
        : -(int32_t)ax->current_vel;
    out->v_max = ax->v_max;
    out->accel = ax->accel;
    out->decel = ax->decel;
    return true;
}

bool motor_is_moving(uint8_t axis)
{
    if (axis >= NUM_AXES) return false;
    axis_state_t st = s_axes[axis].state;
    return (st == AXIS_ACCEL || st == AXIS_CRUISE ||
            st == AXIS_DECEL || st == AXIS_HOMING);
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

bool motor_set_idle_timeout(uint32_t timeout_ms)
{
    s_idle_timeout_ms = timeout_ms;
    return true;
}

/* ================================================================== */
/*  Phase 1 テスト API — デバッグ用                                     */
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
        rmt_stop_channel(ax);
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
    rmt_stop_channel(ax);
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
