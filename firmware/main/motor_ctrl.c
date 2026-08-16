#include "motor_ctrl.h"
#include "adc_monitor.h"
#include "config.h"
#include "encoder.h"
#include "gear_monitor.h"
#include "gpio_config.h"

#include <limits.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
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
#define DEFAULT_HOLD_CURRENT_PERCENT  30U

/* ------------------------------------------------------------------ */
/*  DRV_EN LEDC (保持電流モードのソフトウェアPWMチョッピング用)             */
/*  DRV_EN はアクティブLow(励磁)のため、"percent" = 励磁時間の割合として    */
/*  扱う (100 = 常時Low/フル励磁, 0 = 常時High/無励磁)。                    */
/* ------------------------------------------------------------------ */
#define DRV_EN_LEDC_TIMER    LEDC_TIMER_0
#define DRV_EN_LEDC_CHANNEL  LEDC_CHANNEL_0
#define DRV_EN_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define DRV_EN_LEDC_RES_BITS LEDC_TIMER_10_BIT
#define DRV_EN_LEDC_FREQ_HZ  20000U   /* 可聴域外、DRV8825巻線インダクタンスに対し十分高速 */
#define DRV_EN_LEDC_MAX_DUTY ((1U << DRV_EN_LEDC_RES_BITS) - 1U)

/* 起動初速: 最初の RMT バッチを短時間で完了させる最低速度 */
#define START_VEL_MIN    50.0f
#define START_VEL_MAX   200.0f

/* ホーミングタイムアウト: 30 秒 */
#define HOME_TIMEOUT_US  30000000LL

/* 脱調検出デフォルト閾値 [counts] */
#define DEFAULT_STALL_TH 512U

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
    uint32_t             motion_v_max;  /* 現在モーション用の実効プロファイル */
    uint32_t             motion_accel;
    uint32_t             motion_decel;

    /* ソフトリミット */
    int32_t              min_pos;
    int32_t              max_pos;

    /* アイドルタイムアウト */
    uint32_t             idle_counter_ms;

    /* VEL 割り込みシーケンス (MOVE 中に VEL を受けた場合: DECEL 後に再起動) */
    bool                 vel_pending;
    float                vel_pending_speed;
    bool                 vel_pending_dir;
    bool                 move_aborted;     /* true → DECEL 完了後に EVT MOVE_ABORTED を送出 */

    /* 脱調検出 */
    uint32_t             stall_fault_th;   /* 正規化後の diff > this → FAULT_STALL */
    int8_t               enc_dir;          /* エンコーダ方向: +1 (正) / -1 (逆接続) */
    uint16_t             mpc_x100;         /* microsteps_per_count × 100 (単位正規化) */

    /* モータータイプ（エンコーダ有無） */
    motor_type_t         motor_type;

    /* ドライバタイプ（外付けドライバはフォトカプラ経由でSTEP/DIR反転） */
    driver_type_t        driver_type;
    bool                  home_pending;    /* true = オープンループホーミングの完了待ち (AXIS_DECEL完了時に確定処理) */

    /* フォルトトレース（10ms間隔リングバッファ、GET FAULT_TRACE 用） */
    fault_trace_sample_t trace_buf[FAULT_TRACE_CAPACITY];
    uint16_t              trace_head;      /* 次に書き込むインデックス */
    uint16_t              trace_count;     /* 有効サンプル数 (<= CAPACITY) */

    /* SYNC_MOVE (F-MOT-11) */
    bool                 sync_pending;     /* true = CommTask がパラメータ設定済み・MotorControlTask の START 待ち */

    /* ホーミング */
    uint8_t              home_phase;       /* 0=coarse, 1=backoff, 2=fine, 3=done */
    int8_t               home_dir;         /* +1 or -1 */
    uint32_t             v_home_coarse;    /* steps/sec */
    uint32_t             v_home_fine;      /* steps/sec */
    int32_t              back_off_steps;
    int32_t              home_offset_steps;
    int32_t              home_backoff_target;
    int64_t              home_timeout_us;  /* 絶対時刻 [µs] (esp_timer_get_time()) */
} axis_t;

static axis_t   s_axes[NUM_AXES];
static bool     s_drv_enabled = false;
static uint32_t s_idle_timeout_ms = DEFAULT_IDLE_TIMEOUT_MS;

/* 保持電流モード（全軸共通、DRV_ENが単一GPIOのため） */
static hold_mode_t s_hold_mode              = HOLD_MODE_NORMAL;
static uint8_t      s_hold_current_percent  = DEFAULT_HOLD_CURRENT_PERCENT;
static bool          s_chopping_active      = false;   /* HOLD_REDUCEDでチョッピング中か */

/* step_pos の ISR ↔ タスク競合を防ぐスピンロック (F-MOT-05 / 9.8) */
static portMUX_TYPE s_step_pos_mux = portMUX_INITIALIZER_UNLOCKED;

/* フォルトトレースリングバッファの MotorControlTask(書込) ↔ CommTask(読出) 競合防止 */
static portMUX_TYPE s_trace_mux = portMUX_INITIALIZER_UNLOCKED;

/* フォルト情報（最後の FAULT を記録） */
static fault_info_t     s_last_fault   = { FAULT_NONE, 0, 0 };

/* イベントコールバック */
static motor_event_cb_t  s_move_done_cb      = NULL;
static motor_event_cb_t  s_move_aborted_cb   = NULL;
static motor_event_cb_t  s_limit_hit_cb      = NULL;
static motor_fault_cb_t  s_fault_cb          = NULL;
static motor_event_cb_t  s_home_done_cb      = NULL;
static motor_event_cb_t  s_home_timeout_cb   = NULL;

/* SYNC_MOVE グループ状態 (F-MOT-11) */
static volatile uint8_t  s_sync_group_mask    = 0;    /* 同期軸ビットマスク */
static volatile bool     s_sync_start_pending = false; /* 開始待ちフラグ */
static volatile bool     s_sync_active        = false; /* 同期実行中 */
static motor_sync_cb_t   s_sync_done_cb       = NULL;
static motor_sync_cb_t   s_sync_aborted_cb    = NULL;

static void finish_open_loop_home(uint8_t axis);

static inline void axis_apply_base_profile(axis_t *ax)
{
    ax->motion_v_max = ax->v_max;
    ax->motion_accel = ax->accel;
    ax->motion_decel = ax->decel;
}

static inline void axis_apply_motion_profile(axis_t *ax, uint32_t v_max,
                                             uint32_t accel, uint32_t decel)
{
    ax->motion_v_max = v_max;
    ax->motion_accel = accel;
    ax->motion_decel = decel;
}

/* ------------------------------------------------------------------ */
/*  ヘルパー: 共有 DRV_EN ラインの実効アクティブレベル判定                  */
/*  オンボード DRV8825 はアクティブLow(EN=0で励磁)。外付けドライバ(TB6600等)*/
/*  は実測によりENA=1(High)でENABLEのため、極性が正反対。同一GPIOを共有   */
/*  するため、いずれかの軸が外付けドライバ(DRIVER_TYPE_EXTERNAL)を使用    */
/*  している場合はライン全体をアクティブHighへ切り替える。               */
/* ------------------------------------------------------------------ */
static bool drv_en_active_high(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        if (s_axes[i].driver_type == DRIVER_TYPE_EXTERNAL) return true;
    }
    return false;
}

static uint8_t s_drv_en_last_percent = 0U;   /* drv_en_active_high() 変化時の再適用用 */

/* ------------------------------------------------------------------ */
/*  ヘルパー: DRV_EN を LEDC 経由で駆動する                              */
/*  percent = 励磁時間の割合。100=常時アクティブ(フル励磁)、0=常時非アクティブ(無励磁) */
/* ------------------------------------------------------------------ */
static void drv_en_set_percent(uint8_t percent)
{
    if (percent > 100U) percent = 100U;
    s_drv_en_last_percent = percent;

    /* active_high: 外部ドライバ使用時はHighデューティ=励磁割合。
       active_low  (既定): DRV8825はアクティブLowのため、Lowデューティ=励磁割合。 */
    uint32_t duty = drv_en_active_high()
        ? (uint32_t)(percent * DRV_EN_LEDC_MAX_DUTY / 100U)
        : (uint32_t)((100U - percent) * DRV_EN_LEDC_MAX_DUTY / 100U);
    ledc_set_duty(DRV_EN_LEDC_MODE, DRV_EN_LEDC_CHANNEL, duty);
    ledc_update_duty(DRV_EN_LEDC_MODE, DRV_EN_LEDC_CHANNEL);
}

/* ------------------------------------------------------------------ */
/*  ヘルパー: 外付けドライバ（フォトカプラ経由）の STEP/DIR 極性反転        */
/*  オンボード DRV8825: STEP アイドル=Low, アクティブパルス=High (直結)     */
/*  外付け(TB6600等): アノード5V/カソード信号 → GPIO Low でLED点灯(有効)   */
/*                     のためアイドル=High, アクティブパルス=Low          */
/* ------------------------------------------------------------------ */
static inline int step_idle_level(const axis_t *ax)
{
    return (ax->driver_type == DRIVER_TYPE_EXTERNAL) ? 1 : 0;
}

static inline int step_active_level(const axis_t *ax)
{
    return (ax->driver_type == DRIVER_TYPE_EXTERNAL) ? 0 : 1;
}

static inline int dir_out_level(const axis_t *ax)
{
    bool level = ax->dir;
    if (ax->driver_type == DRIVER_TYPE_EXTERNAL) level = !level;
    return level ? 1 : 0;
}

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
        .flags.eot_level = step_idle_level(ax),
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
/*  フォルトトレース記録 (10ms毎、全軸)                                  */
/*  診断用の参考値のため、state/pos/enc間で厳密な同時刻性は保証しない。   */
/* ------------------------------------------------------------------ */
static void fault_trace_record_all(void)
{
    float current_mA = adc_get_current_mA();
    int16_t current_i16 = (int16_t)(current_mA > 32767.0f ? 32767.0f
        : (current_mA < -32768.0f ? -32768.0f : current_mA));

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        axis_t *ax = &s_axes[i];

        int32_t cur_pos;
        taskENTER_CRITICAL(&s_step_pos_mux);
        cur_pos = ax->step_pos;
        taskEXIT_CRITICAL(&s_step_pos_mux);

        int32_t enc_steps = 0;
        int32_t diff      = 0;
        if (ax->motor_type == MOTOR_TYPE_CLOSED_LOOP) {
            int32_t enc_pos = encoder_get_pos(i);
            enc_steps = (ax->mpc_x100 > 0)
                ? (int32_t)((int64_t)enc_pos * (int64_t)ax->enc_dir
                            * (int64_t)ax->mpc_x100 / 100LL)
                : (int32_t)((int64_t)enc_pos * (int64_t)ax->enc_dir);
            diff = enc_steps - cur_pos;
        }
        int32_t vel = ax->dir ? (int32_t)ax->current_vel : -(int32_t)ax->current_vel;

        fault_trace_sample_t sample = {
            .state      = (uint8_t)ax->state,
            .step_pos   = cur_pos,
            .enc_steps  = enc_steps,   /* オープンループ軸は常に0（未計装） */
            .diff       = diff,
            .vel        = vel,
            .current_mA = current_i16,
        };

        taskENTER_CRITICAL(&s_trace_mux);
        ax->trace_buf[ax->trace_head] = sample;
        ax->trace_head = (uint16_t)((ax->trace_head + 1) % FAULT_TRACE_CAPACITY);
        if (ax->trace_count < FAULT_TRACE_CAPACITY) ax->trace_count++;
        taskEXIT_CRITICAL(&s_trace_mux);
    }
}

/* ------------------------------------------------------------------ */
/*  MotorControlTask — 1ms ティック台形プロファイルエンジン              */
/* ------------------------------------------------------------------ */
static void motor_control_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   enc_tick  = 0;   /* encoder_update_10ms 用カウンタ */

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));

        /* --- encoder 速度更新 (10ms 毎) --- */
        if (++enc_tick >= 10) {
            enc_tick = 0;
            encoder_update_10ms();
            fault_trace_record_all();
        }

        /* --- SYNC_MOVE 開始フェーズ（F-MOT-11）: 同一ティック先頭で全 RMT を同時 enable --- */
        if (s_sync_start_pending) {
            s_sync_start_pending = false;
            s_sync_active        = true;
            for (int i = 0; i < NUM_AXES; i++) {
                if (s_axes[i].sync_pending) {
                    s_axes[i].sync_pending = false;
                    s_axes[i].running      = true;
                    rmt_kick(&s_axes[i]);
                }
            }
        }

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

        if (s_hold_mode != HOLD_MODE_NORMAL) {
            /* HOLD_FULL / HOLD_REDUCED: アイドルタイムアウトによる無励磁遷移は行わない。
               全軸アイドル中(!any_active)はHOLD_REDUCEDならDRV_ENをチョッピングし、
               いずれかの軸が動き出した瞬間に即フル励磁へ復帰する。 */
            for (int i = 0; i < NUM_AXES; i++) s_axes[i].idle_counter_ms = 0;
            if (s_drv_enabled) {
                if (s_hold_mode == HOLD_MODE_REDUCED && !any_active && !s_chopping_active) {
                    drv_en_set_percent(s_hold_current_percent);
                    s_chopping_active = true;
                } else if (s_chopping_active &&
                           (any_active || s_hold_mode != HOLD_MODE_REDUCED)) {
                    drv_en_set_percent(100);
                    s_chopping_active = false;
                }
            } else if (s_chopping_active) {
                s_chopping_active = false;
            }
        }

        for (int i = 0; i < NUM_AXES; i++) {
            axis_t *ax = &s_axes[i];

            /* --- アイドルカウンタ (NORMAL モードのみ有効、HOLD_FULL/HOLD_REDUCED は
                   上のブロックで一括処理済みのためスキップ) --- */
            if (s_hold_mode == HOLD_MODE_NORMAL) {
                if (ax->state == AXIS_IDLE) {
                    if (any_active) {
                        /* DRV_EN / DRV_SLEEP は全軸共通。他軸の動作中は
                           「全軸アイドル」時間に算入しない。 */
                        ax->idle_counter_ms = 0;
                    } else {
                        ax->idle_counter_ms++;
                    }
                    if (!any_active && s_idle_timeout_ms > 0 &&
                        ax->idle_counter_ms >= s_idle_timeout_ms) {
                        /* 全軸アイドル到達 → スリープ遷移（全軸共通信号は1回だけ操作） */
                        if (s_drv_enabled) {
                            drv_en_set_percent(0);              /* コイル解除 */
                            gpio_set_level(GPIO_DRV_SLEEP, 0);  /* スリープ   */
                            s_drv_enabled = false;
                            s_chopping_active = false;
                        }
                        ax->state = AXIS_SLEEP;
                        ax->idle_counter_ms = 0;
                        ESP_LOGI(TAG, "Axis %d: idle timeout → SLEEP", i);
                    }
                } else if (ax->state != AXIS_SLEEP && ax->state != AXIS_FAULT) {
                    ax->idle_counter_ms = 0;
                }
            }

            /* --- 速度プロファイル更新 / HOMING 以外はスキップ --- */
            if (ax->state != AXIS_ACCEL &&
                ax->state != AXIS_CRUISE &&
                ax->state != AXIS_DECEL  &&
                ax->state != AXIS_HOMING) {
                continue;
            }

            /* 現在ステップ位置読み取り (ISR と共有) */
            int32_t cur_pos;
            taskENTER_CRITICAL(&s_step_pos_mux);
            cur_pos = ax->step_pos;
            taskEXIT_CRITICAL(&s_step_pos_mux);

            /* --- 脱調検出 (ACCEL/CRUISE/DECEL 中のみ、エンコーダ付き軸のみ) --- */
            if (ax->motor_type == MOTOR_TYPE_CLOSED_LOOP &&
                (ax->state == AXIS_ACCEL || ax->state == AXIS_CRUISE ||
                 ax->state == AXIS_DECEL)) {
                int32_t enc_pos = encoder_get_pos((uint8_t)i);
                /* enc_pos (counts) を motor_steps 単位に正規化して比較
                   enc_steps = enc_pos × enc_dir × (microsteps_per_count)
                             = enc_pos × enc_dir × mpc_x100 / 100              */
                int32_t enc_steps = (ax->mpc_x100 > 0)
                    ? (int32_t)((int64_t)enc_pos * (int64_t)ax->enc_dir
                                * (int64_t)ax->mpc_x100 / 100LL)
                    : (int32_t)((int64_t)enc_pos * (int64_t)ax->enc_dir);
                int32_t diff = enc_steps - cur_pos;
                if (diff < 0) diff = -diff;
                if ((uint32_t)diff > ax->stall_fault_th) {
                    ESP_LOGE(TAG, "Axis %d: STALL enc=%ld enc_steps=%ld step=%ld diff=%ld",
                             i, (long)enc_pos, (long)enc_steps, (long)cur_pos, (long)diff);
                    motor_estop(FAULT_STALL);
                    continue;
                }
            }

            /* --- HOMING 状態機械 --- */
            if (ax->state == AXIS_HOMING) {
                /* タイムアウト監視 */
                if (ax->running &&
                    esp_timer_get_time() >= ax->home_timeout_us) {
                    rmt_stop_channel(ax);
                    ax->running     = false;
                    ax->current_vel = 0.0f;
                    motor_estop(FAULT_ESTOP);
                    ESP_LOGE(TAG, "Axis %d: HOME timeout", i);
                    if (s_home_timeout_cb) s_home_timeout_cb((uint8_t)i);
                    continue;
                }

                switch (ax->home_phase) {
                case 0: /* coarse: Z イベント待ち */
                    if (encoder_take_z_event((uint8_t)i)) {
                        rmt_stop_channel(ax);
                        ax->running     = false;
                        ax->current_vel = 0.0f;
                        ax->home_phase  = 1;
                        /* 次ティックでバックオフ開始 (DIR セットアップ時間確保) */
                    }
                    break;

                case 1: /* backoff */
                    if (!ax->running) {
                        /* バックオフ開始: home_dir 逆方向に back_off_steps */
                        ax->dir = !(ax->home_dir > 0);
                        gpio_set_level(s_dir_gpios[i], dir_out_level(ax));
                        /* DIR セットアップ: RMT HIGH 期間 2µs で 650ns 要件を満たす */
                        ax->home_backoff_target = cur_pos +
                            (int32_t)ax->back_off_steps *
                            (ax->home_dir > 0 ? -1 : 1);
                        ax->current_vel = (float)ax->v_home_fine;
                        set_rmt_freq(ax, ax->current_vel);
                        ax->running = true;
                    } else {
                        bool done = ax->dir
                            ? (cur_pos >= ax->home_backoff_target)
                            : (cur_pos <= ax->home_backoff_target);
                        if (done) {
                            rmt_stop_channel(ax);
                            ax->running     = false;
                            ax->current_vel = 0.0f;
                            ax->home_phase  = 2;
                            /* 次ティックで fine homing 開始 */
                        }
                    }
                    break;

                case 2: /* fine: Z イベント待ち */
                    if (!ax->running) {
                        /* fine homing 開始: home_dir 方向 */
                        ax->dir = (ax->home_dir > 0);
                        gpio_set_level(s_dir_gpios[i], dir_out_level(ax));
                        encoder_take_z_event((uint8_t)i); /* 残留イベントクリア */
                        ax->current_vel = (float)ax->v_home_fine;
                        set_rmt_freq(ax, ax->current_vel);
                        ax->running = true;
                    } else if (encoder_take_z_event((uint8_t)i)) {
                        rmt_stop_channel(ax);
                        ax->running     = false;
                        ax->current_vel = 0.0f;
                        /* step_pos と encoder_pos をホームオフセットにセット */
                        taskENTER_CRITICAL(&s_step_pos_mux);
                        ax->step_pos = ax->home_offset_steps;
                        taskEXIT_CRITICAL(&s_step_pos_mux);
                        encoder_set_pos((uint8_t)i, ax->home_offset_steps);
                        ax->home_phase      = 3;
                        ax->state           = AXIS_IDLE;
                        ax->idle_counter_ms = 0;
                        ESP_LOGI(TAG, "Axis %d: HOME done offset=%ld",
                                 i, (long)ax->home_offset_steps);
                        if (s_home_done_cb) s_home_done_cb((uint8_t)i);
                    }
                    break;

                default:
                    ax->state = AXIS_IDLE;
                    break;
                }

                if (ax->running) rmt_kick(ax);
                continue;
            }

            /* --- 残りステップ (位置モード) --- */
            int32_t remaining = 0;
            if (ax->pos_mode) {
                remaining = ax->dir
                    ? (ax->target_pos - cur_pos)
                    : (cur_pos - ax->target_pos);
            }

            /* VEL モード: ソフトリミット到達チェック */
            if (!ax->pos_mode &&
                ax->state != AXIS_DECEL && ax->state != AXIS_FAULT) {
                if ((ax->dir  && cur_pos >= ax->max_pos) ||
                    (!ax->dir && cur_pos <= ax->min_pos)) {
                    ESP_LOGW(TAG, "Axis %d: soft limit at pos=%ld", i, (long)cur_pos);
                    if (s_sync_active && (s_sync_group_mask & (1u << i))) {
                        /* SYNC_MOVE 中: グループ全軸を台形減速停止 (F-MOT-11) */
                        uint8_t mask = s_sync_group_mask;
                        for (int j = 0; j < NUM_AXES; j++) {
                            if ((s_sync_group_mask & (1u << j)) && s_axes[j].running) {
                                s_axes[j].pos_mode        = false;
                                s_axes[j].disable_on_stop = false;
                                s_axes[j].state           = AXIS_DECEL;
                            }
                        }
                        s_sync_group_mask = 0;
                        s_sync_active     = false;
                        if (s_sync_aborted_cb) s_sync_aborted_cb(mask);
                        ESP_LOGW(TAG, "SYNC_ABORTED: limit hit on axis %d", i);
                    } else {
                        ax->state    = AXIS_DECEL;
                        ax->pos_mode = false;
                    }
                    if (s_limit_hit_cb) s_limit_hit_cb((uint8_t)i);
                }
            }

            /* 現在速度で停止するのに必要なステップ数 */
            float decel_steps = (ax->current_vel * ax->current_vel) /
                                 (2.0f * (float)ax->motion_decel);

            switch (ax->state) {
            case AXIS_ACCEL: {
                ax->current_vel += (float)ax->motion_accel * 0.001f;
                float vel_cap = ax->pos_mode
                    ? (float)ax->motion_v_max
                    : (ax->target_vel < (float)ax->motion_v_max ? ax->target_vel : (float)ax->motion_v_max);
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
                        ax->current_vel += (float)ax->motion_accel * 0.001f;
                        if (ax->current_vel > tv) ax->current_vel = tv;
                        set_rmt_freq(ax, ax->current_vel);
                    } else if (ax->current_vel > tv) {
                        ax->current_vel -= (float)ax->motion_decel * 0.001f;
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
                ax->current_vel -= (float)ax->motion_decel * 0.001f;
                if (ax->current_vel < 1.0f ||
                    (ax->pos_mode && remaining <= 0)) {
                    ax->current_vel = 0.0f;
                    ax->running     = false;
                    rmt_stop_channel(ax);   /* キューに残ったバッチを即時消去 */

                    bool was_pos_mode = ax->pos_mode;
                    bool was_aborted  = ax->move_aborted;
                    ax->move_aborted  = false;

                    if (ax->vel_pending) {
                        /* VEL 割り込みシーケンス: 停止後に VEL モード再起動 */
                        ax->vel_pending     = false;
                        ax->dir             = ax->vel_pending_dir;
                        ax->target_vel      = ax->vel_pending_speed;
                        ax->pos_mode        = false;
                        ax->current_vel     = START_VEL_MIN;
                        ax->disable_on_stop = false;

                        gpio_set_level(s_dir_gpios[i], dir_out_level(ax));
                        /* DIR セットアップ: RMT 最初の LOW 期間（2µs）で 650ns 保証 */

                        ax->state           = AXIS_ACCEL;
                        ax->running         = true;
                        ax->idle_counter_ms = 0;
                        set_rmt_freq(ax, ax->current_vel);
                        /* ax->running = true のためループ末尾で rmt_kick() が実行される */
                    } else {
                        ax->state           = AXIS_IDLE;
                        ax->idle_counter_ms = 0;

                        if (ax->disable_on_stop) {
                            ax->disable_on_stop = false;
                            drv_en_set_percent(0);
                            s_drv_enabled = false;
                            s_chopping_active = false;
                        }

                        ESP_LOGI(TAG, "Axis %d: done, pos=%ld", i, (long)ax->step_pos);

                        if (ax->home_pending) {
                            finish_open_loop_home((uint8_t)i);
                        } else if (was_pos_mode && s_move_done_cb) {
                            s_move_done_cb((uint8_t)i);
                        }

                        /* SYNC_MOVE 完了チェック: グループ全軸が IDLE になったか確認 (F-MOT-11) */
                        if (s_sync_active && (s_sync_group_mask & (1u << i))) {
                            bool all_done = true;
                            for (int j = 0; j < NUM_AXES; j++) {
                                if ((s_sync_group_mask & (1u << j)) &&
                                    s_axes[j].state != AXIS_IDLE) {
                                    all_done = false;
                                    break;
                                }
                            }
                            if (all_done) {
                                uint8_t mask  = s_sync_group_mask;
                                s_sync_group_mask = 0;
                                s_sync_active     = false;
                                ESP_LOGI(TAG, "SYNC_DONE mask=0x%02x", mask);
                                if (s_sync_done_cb) s_sync_done_cb(mask);
                            }
                        }
                    }

                    /* MOVE 中断通知（vel_pending 再起動の有無に関わらず送出） */
                    if (was_aborted && s_move_aborted_cb) {
                        s_move_aborted_cb((uint8_t)i);
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
    /* DRV_EN を LEDC (ハードウェアPWM) で駆動する設定。main.c の起動安全初期化
       (raw gpio_set_level, 無励磁High) の後にここで一度だけ設定し、duty=0
       (=無励磁High継続) から開始してグリッチなく引き継ぐ。 */
    const ledc_timer_config_t drv_en_timer_cfg = {
        .speed_mode      = DRV_EN_LEDC_MODE,
        .duty_resolution = DRV_EN_LEDC_RES_BITS,
        .timer_num       = DRV_EN_LEDC_TIMER,
        .freq_hz         = DRV_EN_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&drv_en_timer_cfg));
    const ledc_channel_config_t drv_en_channel_cfg = {
        .gpio_num   = GPIO_DRV_EN,
        .speed_mode = DRV_EN_LEDC_MODE,
        .channel    = DRV_EN_LEDC_CHANNEL,
        .timer_sel  = DRV_EN_LEDC_TIMER,
        .duty       = DRV_EN_LEDC_MAX_DUTY,   /* 初期状態は全軸ONBOARD(アクティブLow)前提: percent=0相当・常時High(無励磁) */
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&drv_en_channel_cfg));

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

        ax->driver_type = DRIVER_TYPE_ONBOARD;
        ax->sym.level0  = step_active_level(ax);
        ax->sym.level1  = step_idle_level(ax);
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
        axis_apply_base_profile(ax);
        ax->min_pos         = -2000000;
        ax->max_pos         =  2000000;
        ax->idle_counter_ms = 0;

        ax->vel_pending       = false;
        ax->vel_pending_speed = 0.0f;
        ax->vel_pending_dir   = true;
        ax->move_aborted      = false;

        ax->stall_fault_th     = DEFAULT_STALL_TH;
        ax->enc_dir            = 1;
        ax->mpc_x100           = 160;   /* ME1K 1000PPR×4 / (200×32 µsteps) = 0.625 → ×100=160 */
        ax->sync_pending       = false;
        ax->motor_type         = MOTOR_TYPE_CLOSED_LOOP;
        ax->home_pending       = false;

        ax->home_phase         = 0;
        ax->home_dir           = -1;
        ax->v_home_coarse      = 2000;
        ax->v_home_fine        = 500;
        ax->back_off_steps     = 200;
        ax->home_offset_steps  = 0;
        ax->home_backoff_target = 0;
        ax->home_timeout_us    = 0;

        ESP_LOGI(TAG, "Axis %d RMT init OK (GPIO %d)", i, s_step_gpios[i]);
    }

    xTaskCreate(motor_control_task, "MotorCtrl", 4096, NULL, 20, NULL);
    ESP_LOGI(TAG, "MotorControlTask started");
}

/* ------------------------------------------------------------------ */
/*  コールバック登録                                                      */
/* ------------------------------------------------------------------ */
void motor_register_move_done_cb(motor_event_cb_t cb)    { s_move_done_cb      = cb; }
void motor_register_move_aborted_cb(motor_event_cb_t cb) { s_move_aborted_cb   = cb; }
void motor_register_limit_hit_cb(motor_event_cb_t cb)    { s_limit_hit_cb      = cb; }
void motor_register_fault_cb(motor_fault_cb_t cb)        { s_fault_cb          = cb; }
void motor_register_home_done_cb(motor_event_cb_t cb)    { s_home_done_cb      = cb; }
void motor_register_home_timeout_cb(motor_event_cb_t cb) { s_home_timeout_cb   = cb; }
void motor_register_sync_done_cb(motor_sync_cb_t cb)     { s_sync_done_cb      = cb; }
void motor_register_sync_aborted_cb(motor_sync_cb_t cb)  { s_sync_aborted_cb   = cb; }

/* ------------------------------------------------------------------ */
/*  motor_enable / motor_disable                                        */
/* ------------------------------------------------------------------ */
void motor_enable(void)
{
    /* 無効化後の再 ENABLE: エンコーダ位置をコマンド位置に再基準化
       (無励磁中のドリフトを受け入れ、脱調誤検出を防止する)        */
    bool resync = !s_drv_enabled;

    gpio_set_level(GPIO_DRV_SLEEP, 1);   /* スリープ解除 */
    vTaskDelay(pdMS_TO_TICKS(1));         /* チャージポンプ安定待ち */
    drv_en_set_percent(100);              /* フル励磁 (極性は drv_en_active_high() に従う) */
    s_drv_enabled = true;
    s_chopping_active = false;

    for (int i = 0; i < NUM_AXES; i++) {
        if (s_axes[i].state == AXIS_SLEEP) {
            s_axes[i].state = AXIS_IDLE;
            s_axes[i].idle_counter_ms = 0;
        }
        if (resync && s_axes[i].motor_type == MOTOR_TYPE_CLOSED_LOOP &&
            s_axes[i].mpc_x100 > 0 && s_axes[i].enc_dir != 0) {
            /* enc_pos = step_pos × 100 / (enc_dir × mpc_x100)
               → enc_steps = enc_pos × enc_dir × mpc_x100 / 100 = step_pos */
            int32_t cur;
            taskENTER_CRITICAL(&s_step_pos_mux);
            cur = s_axes[i].step_pos;
            taskEXIT_CRITICAL(&s_step_pos_mux);
            int32_t enc_pos = (int32_t)((int64_t)cur * 100LL
                              / ((int64_t)s_axes[i].enc_dir
                                 * (int64_t)s_axes[i].mpc_x100));
            encoder_set_pos((uint8_t)i, enc_pos);
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
        gpio_set_level(s_step_gpios[i], step_idle_level(ax));
    }
    drv_en_set_percent(0);
    s_drv_enabled = false;
    s_chopping_active = false;
    ESP_LOGI(TAG, "DRV disabled");
}

bool motor_is_enabled(void)
{
    return s_drv_enabled;
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
    gpio_set_level(s_dir_gpios[axis], dir_out_level(ax));
    vTaskDelay(pdMS_TO_TICKS(1));

    ax->target_pos      = target;
    ax->pos_mode        = pos_mode;
    ax->disable_on_stop = false;
    ax->idle_counter_ms = 0;
    axis_apply_base_profile(ax);

    /* 初速: 最初のバッチを短時間で完了させる最低速度 */
    float sv = (float)ax->motion_v_max * 0.1f;
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
bool motor_degrees_to_steps(uint8_t axis, float deg, int32_t *out_steps)
{
    if (axis >= NUM_AXES || !out_steps || !isfinite(deg)) return false;

    uint32_t steps_per_rev = config_get_steps_per_rev(axis);
    float gear_ratio = config_get_gear_ratio(axis);
    if (steps_per_rev == 0 || !isfinite(gear_ratio) || gear_ratio <= 0.0f) {
        return false;
    }

    double steps = round((double)deg * (double)steps_per_rev *
                         (double)gear_ratio / 360.0);
    if (steps < (double)INT32_MIN || steps > (double)INT32_MAX) return false;
    *out_steps = (int32_t)steps;
    return true;
}

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
    if (speed > (float)ax->v_max) speed = (float)ax->v_max;

    if (ax->running && ax->state != AXIS_FAULT) {
        /* 既存 VEL モード（同方向）: 目標速度だけ更新 */
        if (!ax->pos_mode && ax->dir == dir) {
            ax->target_vel = speed;
            ax->state      = AXIS_CRUISE;
            return true;
        }
        /* MOVE/MOVETO 実行中 or 逆方向: 減速停止後に VEL モードへ移行 */
        bool was_pos          = ax->pos_mode;
        ax->pos_mode          = false;
        ax->home_pending      = false;   /* オープンループホーミング中の割り込みは中断扱い */
        ax->vel_pending       = true;
        ax->vel_pending_speed = speed;
        ax->vel_pending_dir   = dir;
        ax->state             = AXIS_DECEL;
        if (was_pos) ax->move_aborted = true;   /* EVT MOVE_ABORTED を予約 */
        return true;
    }

    /* 停止中: 新規 VEL 開始 */
    ax->target_vel = speed;
    axis_apply_base_profile(ax);
    return start_motion(axis, dir, 0, false);
}

/* ------------------------------------------------------------------ */
/*  motor_stop / motor_stop_free / motor_estop                         */
/* ------------------------------------------------------------------ */
bool motor_stop(uint8_t axis)
{
    if (axis >= NUM_AXES) return false;
    axis_t *ax = &s_axes[axis];
    if (!ax->running) return true;

    /* SYNC_MOVE 中: グループ全軸を台形減速停止 (F-MOT-11) */
    if (s_sync_active && (s_sync_group_mask & (1u << axis))) {
        uint8_t mask = s_sync_group_mask;
        for (int j = 0; j < NUM_AXES; j++) {
            if ((s_sync_group_mask & (1u << j)) && s_axes[j].running) {
                s_axes[j].pos_mode        = false;
                s_axes[j].disable_on_stop = false;
                s_axes[j].state           = AXIS_DECEL;
            }
        }
        s_sync_group_mask = 0;
        s_sync_active     = false;
        if (s_sync_aborted_cb) s_sync_aborted_cb(mask);
        ESP_LOGI(TAG, "SYNC_ABORTED via STOP axis %d", axis);
        return true;
    }

    ax->pos_mode        = false;
    ax->disable_on_stop = false;
    ax->home_pending    = false;   /* オープンループホーミング中の STOP は中断扱い */
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
        drv_en_set_percent(0);
        s_drv_enabled = false;
        s_chopping_active = false;
        return true;
    }

    ax->pos_mode        = false;
    ax->disable_on_stop = true;   /* DECEL 完了後にコイル解除 */
    ax->home_pending    = false;  /* オープンループホーミング中の STOP は中断扱い */
    ax->state           = AXIS_DECEL;
    ESP_LOGI(TAG, "Axis %d: STOP_FREE (decel then disable)", axis);
    return true;
}

void motor_estop(fault_reason_t reason)
{
    /* SYNC_MOVE 実行中であれば中断通知（FAULT cb の前に送出） */
    if (s_sync_active) {
        uint8_t smask = s_sync_group_mask;
        s_sync_group_mask    = 0;
        s_sync_active        = false;
        s_sync_start_pending = false;
        if (s_sync_aborted_cb) s_sync_aborted_cb(smask);
    }

    uint8_t mask = 0;
    for (int i = 0; i < NUM_AXES; i++) {
        axis_t *ax = &s_axes[i];
        if (ax->state != AXIS_SLEEP) {
            mask |= (1u << i);
        }
        ax->running      = false;
        ax->current_vel  = 0.0f;
        ax->state        = AXIS_FAULT;
        ax->sync_pending = false;
        ax->home_pending = false;
        rmt_stop_channel(ax);
        gpio_set_level(s_step_gpios[i], step_idle_level(ax));
    }

    /* DRV_EN → High（コイル励磁解除）: FAULT 遷移処理 手順2 */
    drv_en_set_percent(0);
    s_drv_enabled = false;
    s_chopping_active = false;

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
    drv_en_set_percent(0);
    s_drv_enabled = false;
    s_chopping_active = false;

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
/*  motor_get_fault_trace  (GET FAULT_TRACE)                           */
/* ------------------------------------------------------------------ */
uint16_t motor_get_fault_trace(uint8_t axis, fault_trace_sample_t *out, uint16_t max_entries)
{
    if (axis >= NUM_AXES || !out || max_entries == 0) return 0;
    axis_t *ax = &s_axes[axis];

    taskENTER_CRITICAL(&s_trace_mux);
    uint16_t count  = ax->trace_count;
    uint16_t oldest = (uint16_t)((ax->trace_head + FAULT_TRACE_CAPACITY - count) % FAULT_TRACE_CAPACITY);
    uint16_t copied = count < max_entries ? count : max_entries;
    /* max_entries が保持件数より小さい場合は新しい方を優先して返す */
    uint16_t skip  = (uint16_t)(count - copied);
    uint16_t start = (uint16_t)((oldest + skip) % FAULT_TRACE_CAPACITY);
    for (uint16_t i = 0; i < copied; i++) {
        out[i] = ax->trace_buf[(start + i) % FAULT_TRACE_CAPACITY];
    }
    taskEXIT_CRITICAL(&s_trace_mux);
    return copied;
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
    s_axes[axis].motion_v_max = vmax;
    return true;
}

bool motor_set_accel(uint8_t axis, uint32_t accel_val)
{
    if (axis >= NUM_AXES || accel_val == 0) return false;
    s_axes[axis].accel = accel_val;
    s_axes[axis].motion_accel = accel_val;
    return true;
}

bool motor_set_decel(uint8_t axis, uint32_t decel_val)
{
    if (axis >= NUM_AXES || decel_val == 0) return false;
    s_axes[axis].decel = decel_val;
    s_axes[axis].motion_decel = decel_val;
    return true;
}

bool motor_set_idle_timeout(uint32_t timeout_ms)
{
    s_idle_timeout_ms = timeout_ms;
    return true;
}

bool motor_set_hold_mode(hold_mode_t mode)
{
    if (mode != HOLD_MODE_NORMAL && mode != HOLD_MODE_FULL && mode != HOLD_MODE_REDUCED) {
        return false;
    }
    s_hold_mode = mode;
    return true;
}

hold_mode_t motor_get_hold_mode(void)
{
    return s_hold_mode;
}

bool motor_set_hold_current_percent(uint8_t percent)
{
    if (percent < 1U || percent > 100U) return false;
    s_hold_current_percent = percent;
    return true;
}

uint8_t motor_get_hold_current_percent(void)
{
    return s_hold_current_percent;
}

bool motor_set_soft_limit(uint8_t axis, int32_t min_p, int32_t max_p)
{
    if (axis >= NUM_AXES || min_p >= max_p) return false;
    s_axes[axis].min_pos = min_p;
    s_axes[axis].max_pos = max_p;
    return true;
}

/* ================================================================== */
/*  Phase 5: SYNC_MOVE (F-MOT-11)                                      */
/* ================================================================== */

bool motor_sync_move(uint8_t n, const uint8_t *axes, const int32_t *steps)
{
    if (n < 2 || n > NUM_AXES) return false;
    if (!s_drv_enabled) {
        ESP_LOGW(TAG, "SYNC_MOVE rejected — call ENABLE first");
        return false;
    }

    /* 参照軸を決定: |steps| が最大の軸 */
    int     ref_idx   = -1;
    int32_t max_steps = 0;
    for (int i = 0; i < (int)n; i++) {
        int32_t abs_s = steps[i] < 0 ? -steps[i] : steps[i];
        if (abs_s > max_steps) {
            max_steps = abs_s;
            ref_idx   = i;
        }
    }
    if (ref_idx < 0 || max_steps == 0) return false;   /* 全軸 steps=0 */

    /* 参照軸パラメータ */
    uint8_t  ref_axis  = axes[ref_idx];
    uint32_t ref_vmax  = s_axes[ref_axis].v_max;
    uint32_t ref_accel = s_axes[ref_axis].accel;
    uint32_t ref_decel = s_axes[ref_axis].decel;

    /* ソフトリミット事前チェック */
    for (int i = 0; i < (int)n; i++) {
        if (steps[i] == 0) continue;
        axis_t *ax = &s_axes[axes[i]];
        taskENTER_CRITICAL(&s_step_pos_mux);
        int32_t cur = ax->step_pos;
        taskEXIT_CRITICAL(&s_step_pos_mux);
        int32_t tgt = cur + steps[i];
        if (tgt > ax->max_pos || tgt < ax->min_pos) return false;  /* ERR E006 */
    }

    /* 全軸パラメータ設定 (まず DIR を全軸セット、その後 vTaskDelay で 650ns 確保) */
    uint8_t mask = 0;
    for (int i = 0; i < (int)n; i++) {
        mask |= (1u << axes[i]);
        if (steps[i] == 0) continue;   /* steps=0 の軸は IDLE 維持 */

        axis_t  *ax  = &s_axes[axes[i]];
        bool     dir = (steps[i] > 0);
        int32_t  abs_s = steps[i] < 0 ? -steps[i] : steps[i];

        /* 速度スケーリング */
        uint32_t vm, ac, dc;
        if (i == ref_idx) {
            vm = ref_vmax;
            ac = ref_accel;
            dc = ref_decel;
        } else {
            float ratio = (float)abs_s / (float)max_steps;
            vm = (uint32_t)((float)ref_vmax  * ratio); if (vm < 1) vm = 1;
            ac = (uint32_t)((float)ref_accel * ratio); if (ac < 1) ac = 1;
            dc = (uint32_t)((float)ref_decel * ratio); if (dc < 1) dc = 1;
        }

        /* DIR GPIO セット（DIR セットアップ時間は直後の vTaskDelay で確保） */
        ax->dir = dir;
        gpio_set_level(s_dir_gpios[axes[i]], dir_out_level(ax));

        /* 現在位置と目標を確定 */
        taskENTER_CRITICAL(&s_step_pos_mux);
        int32_t cur_pos = ax->step_pos;
        taskEXIT_CRITICAL(&s_step_pos_mux);
        ax->target_pos      = cur_pos + steps[i];
        ax->pos_mode        = true;
        ax->disable_on_stop = false;
        ax->idle_counter_ms = 0;

        /* スケール済みパラメータはこの同期移動にだけ適用 */
        axis_apply_motion_profile(ax, vm, ac, dc);

        /* 初速 */
        float sv = (float)ax->motion_v_max * 0.1f;
        if (sv < START_VEL_MIN) sv = START_VEL_MIN;
        if (sv > START_VEL_MAX) sv = START_VEL_MAX;
        ax->current_vel = sv;
        set_rmt_freq(ax, sv);

        ax->state        = AXIS_ACCEL;
        ax->running      = false;   /* MotorControlTask の START フェーズで true に */
        ax->sync_pending = true;
    }

    /* DIR セットアップ時間（DRV8825: 650 ns 以上、1 ms で十分）*/
    vTaskDelay(pdMS_TO_TICKS(1));

    /* グループマスクと開始ペンディングフラグをセット */
    s_sync_group_mask    = mask;
    s_sync_start_pending = true;

    ESP_LOGI(TAG, "SYNC_MOVE queued: n=%u mask=0x%02x ref_axis=%u", n, mask, ref_axis);
    return true;
}

/* ================================================================== */
/*  追加ゲッター（NVS 永続化・状態確認用）                               */
/* ================================================================== */

uint32_t motor_get_stall_fault_th(uint8_t axis)
{
    if (axis >= NUM_AXES) return 0;
    return s_axes[axis].stall_fault_th;
}

bool motor_get_soft_limit(uint8_t axis, int32_t *out_min, int32_t *out_max)
{
    if (axis >= NUM_AXES) return false;
    if (out_min) *out_min = s_axes[axis].min_pos;
    if (out_max) *out_max = s_axes[axis].max_pos;
    return true;
}

uint32_t motor_get_v_home_coarse(uint8_t axis)
{
    if (axis >= NUM_AXES) return 0;
    return s_axes[axis].v_home_coarse;
}

uint32_t motor_get_v_home_fine(uint8_t axis)
{
    if (axis >= NUM_AXES) return 0;
    return s_axes[axis].v_home_fine;
}

int32_t motor_get_back_off_steps(uint8_t axis)
{
    if (axis >= NUM_AXES) return 0;
    return s_axes[axis].back_off_steps;
}

int32_t motor_get_home_offset_steps(uint8_t axis)
{
    if (axis >= NUM_AXES) return 0;
    return s_axes[axis].home_offset_steps;
}

int8_t motor_get_home_dir(uint8_t axis)
{
    if (axis >= NUM_AXES) return -1;
    return s_axes[axis].home_dir;
}

bool motor_set_home_params(uint8_t axis, uint32_t v_coarse, uint32_t v_fine,
                            int32_t back_off, int32_t home_offset)
{
    if (axis >= NUM_AXES) return false;
    s_axes[axis].v_home_coarse     = v_coarse;
    s_axes[axis].v_home_fine       = v_fine;
    s_axes[axis].back_off_steps    = back_off;
    s_axes[axis].home_offset_steps = home_offset;
    return true;
}

/* ================================================================== */
/*  Phase 3: ホーミング / 脱調検出                                      */
/* ================================================================== */

/* オープンループ軸（motor_type == MOTOR_TYPE_OPEN_LOOP）のホーミング完了処理。
   multi_i2c_bridge (AS5600) の絶対角度で校正済みの角度0を home_offset_steps に
   割り当て、蓄積したオープンループ位置ドリフトを補正する。 */
static void finish_open_loop_home(uint8_t axis)
{
    axis_t *ax = &s_axes[axis];

    taskENTER_CRITICAL(&s_step_pos_mux);
    ax->step_pos = ax->home_offset_steps;
    taskEXIT_CRITICAL(&s_step_pos_mux);

    ax->home_pending    = false;
    ax->idle_counter_ms = 0;

    gear_monitor_mark_home(axis);

    ESP_LOGI(TAG, "Axis %d: OPEN_LOOP HOME done offset=%ld",
             axis, (long)ax->home_offset_steps);
    if (s_home_done_cb) s_home_done_cb(axis);
}

/* オープンループ軸のホーミング開始。gear_monitor (AS5600) が示す出力軸絶対角度
   (角度0 = 較正済み原点。「0位置設定」機能で磁石取付誤差は補正済み) へ向けて
   通常のMOVETOと同じ台形プロファイルで移動する。完了は AXIS_DECEL 完了検知
   (home_pending フラグ) 経由で finish_open_loop_home() が担う。 */
static bool start_open_loop_home(uint8_t axis)
{
    axis_t *ax = &s_axes[axis];

    gear_axis_status_t gs;
    if (!gear_monitor_get_axis_status(axis, &gs) || !gs.enabled || !gs.ok) {
        ESP_LOGW(TAG, "Axis %d: HOME rejected — gear angle sensor not ready", axis);
        return false;
    }

    /* angle_deg は 0-360 の較正済み絶対角。最短方向で 0 へ向かう差分 (-180..180]。 */
    float delta_deg = gs.angle_deg;
    if (delta_deg > 180.0f) delta_deg -= 360.0f;

    int32_t delta_steps;
    if (!motor_degrees_to_steps(axis, -delta_deg, &delta_steps)) {
        ESP_LOGW(TAG, "Axis %d: HOME rejected — invalid step/gear ratio config", axis);
        return false;
    }

    taskENTER_CRITICAL(&s_step_pos_mux);
    int32_t cur = ax->step_pos;
    taskEXIT_CRITICAL(&s_step_pos_mux);
    int32_t target = cur + delta_steps;

    ax->home_phase   = 0;
    ax->home_pending = true;

    if (!start_motion(axis, delta_steps >= 0, target, true)) {
        ax->home_pending = false;
        return false;
    }

    if (!ax->running) {
        /* start_motion() が「既に目標位置」として即時 return した場合
           (delta_steps が丸めで 0 になったケース): 同期的に完了処理する。 */
        finish_open_loop_home(axis);
    }

    ESP_LOGI(TAG, "Axis %d: OPEN_LOOP HOMING start angle=%.2f delta_steps=%ld",
             axis, (double)gs.angle_deg, (long)delta_steps);
    return true;
}

bool motor_home(uint8_t axis)
{
    if (axis >= NUM_AXES) return false;
    axis_t *ax = &s_axes[axis];

    if (ax->state == AXIS_FAULT) return false;
    if (motor_is_moving(axis))   return false;
    if (!s_drv_enabled) {
        ESP_LOGW(TAG, "Axis %d: HOME rejected — call ENABLE first", axis);
        return false;
    }

    if (ax->motor_type == MOTOR_TYPE_OPEN_LOOP) {
        return start_open_loop_home(axis);
    }

    /* 残留 Z イベントをクリアして誤検知を防ぐ */
    encoder_take_z_event(axis);

    ax->home_phase          = 0;
    ax->home_backoff_target = 0;
    ax->home_timeout_us     = esp_timer_get_time() + HOME_TIMEOUT_US;
    ax->pos_mode            = false;
    ax->idle_counter_ms     = 0;

    /* DIR 設定 */
    ax->dir = (ax->home_dir > 0);
    gpio_set_level(s_dir_gpios[axis], dir_out_level(ax));
    vTaskDelay(pdMS_TO_TICKS(1));   /* DIR セットアップ時間 */

    ax->current_vel = (float)ax->v_home_coarse;
    set_rmt_freq(ax, ax->current_vel);

    /* state を HOMING に設定してから running を true にする */
    ax->state   = AXIS_HOMING;
    ax->running = true;

    esp_err_t err = rmt_kick(ax);
    if (err != ESP_OK) {
        ax->running = false;
        ax->state   = AXIS_IDLE;
        ESP_LOGE(TAG, "Axis %d home rmt_kick: %s", axis, esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Axis %d: HOMING start dir=%+d coarse=%lu fine=%lu",
             axis, (int)ax->home_dir,
             (unsigned long)ax->v_home_coarse,
             (unsigned long)ax->v_home_fine);
    return true;
}

bool motor_set_stall_fault_th(uint8_t axis, uint32_t th)
{
    if (axis >= NUM_AXES) return false;
    s_axes[axis].stall_fault_th = th;
    return true;
}

int8_t motor_get_enc_dir(uint8_t axis)
{
    if (axis >= NUM_AXES) return 1;
    return s_axes[axis].enc_dir;
}

bool motor_set_enc_dir(uint8_t axis, int8_t dir)
{
    if (axis >= NUM_AXES) return false;
    if (dir != 1 && dir != -1) return false;
    s_axes[axis].enc_dir = dir;
    return true;
}

motor_type_t motor_get_motor_type(uint8_t axis)
{
    if (axis >= NUM_AXES) return MOTOR_TYPE_CLOSED_LOOP;
    return s_axes[axis].motor_type;
}

bool motor_set_motor_type(uint8_t axis, motor_type_t type)
{
    if (axis >= NUM_AXES) return false;
    if (type != MOTOR_TYPE_CLOSED_LOOP && type != MOTOR_TYPE_OPEN_LOOP) return false;
    if (motor_is_moving(axis)) return false;   /* 動作中の切替は禁止 */
    s_axes[axis].motor_type   = type;
    s_axes[axis].home_pending = false;
    return true;
}

driver_type_t motor_get_driver_type(uint8_t axis)
{
    if (axis >= NUM_AXES) return DRIVER_TYPE_ONBOARD;
    return s_axes[axis].driver_type;
}

bool motor_set_driver_type(uint8_t axis, driver_type_t type)
{
    if (axis >= NUM_AXES) return false;
    if (type != DRIVER_TYPE_ONBOARD && type != DRIVER_TYPE_EXTERNAL) return false;
    if (motor_is_moving(axis)) return false;   /* 動作中の切替は禁止 */

    axis_t *ax = &s_axes[axis];
    ax->driver_type = type;

    /* STEP パルス極性・DIR 出力レベルを新しい極性へ即時反映 */
    ax->sym.level0 = step_active_level(ax);
    ax->sym.level1 = step_idle_level(ax);
    if (!ax->running) {
        gpio_set_level(s_step_gpios[axis], step_idle_level(ax));
        gpio_set_level(s_dir_gpios[axis], dir_out_level(ax));
    }

    /* 共有 DRV_EN の実効アクティブレベルが変わった可能性があるため、
       現在の励磁割合を新しい極性で再適用する。 */
    drv_en_set_percent(s_drv_en_last_percent);
    return true;
}

uint16_t motor_get_mpc_x100(uint8_t axis)
{
    if (axis >= NUM_AXES) return 160;
    return s_axes[axis].mpc_x100;
}

bool motor_set_mpc_x100(uint8_t axis, uint16_t mpc)
{
    if (axis >= NUM_AXES) return false;
    s_axes[axis].mpc_x100 = mpc;
    return true;
}

bool motor_set_home_dir(uint8_t axis, int8_t dir)
{
    if (axis >= NUM_AXES) return false;
    if (dir != 1 && dir != -1) return false;
    s_axes[axis].home_dir = dir;
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

    ax->sym.level0    = step_active_level(ax);
    ax->sym.duration0 = (uint16_t)STEP_HIGH_TICKS;
    ax->sym.level1    = step_idle_level(ax);
    ax->sym.duration1 = (uint16_t)low_ticks;
    ax->running       = true;

    const rmt_transmit_config_t tx_cfg = {
        .loop_count      = RMT_LOOP_COUNT,
        .flags.eot_level = step_idle_level(ax),
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
    gpio_set_level(s_step_gpios[axis], step_idle_level(ax));

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
