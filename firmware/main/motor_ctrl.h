#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  フォルト原因                                                         */
/* ------------------------------------------------------------------ */
typedef enum {
    FAULT_NONE        = 0,
    FAULT_ESTOP       = 1,
    FAULT_OVERCURRENT = 2,
    FAULT_STALL       = 3,
} fault_reason_t;

/* ------------------------------------------------------------------ */
/*  軸状態                                                              */
/* ------------------------------------------------------------------ */
typedef enum {
    AXIS_SLEEP = 0,
    AXIS_IDLE,
    AXIS_ACCEL,
    AXIS_CRUISE,
    AXIS_DECEL,
    AXIS_HOMING,
    AXIS_FAULT,
} axis_state_t;

/* ------------------------------------------------------------------ */
/*  モータータイプ（軸ごと。エンコーダの有無を指定する）                    */
/* ------------------------------------------------------------------ */
typedef enum {
    MOTOR_TYPE_CLOSED_LOOP = 0,   /* エンコーダ付き（既定・脱調検出/Z相ホーミング有効） */
    MOTOR_TYPE_OPEN_LOOP   = 1,   /* エンコーダなし（脱調検出無効、multi_i2c_bridge 絶対角でホーミング） */
} motor_type_t;

typedef struct {
    axis_state_t state;
    int32_t      pos;          /* 現在位置 [steps] */
    int32_t      vel;          /* 符号付き速度 [steps/sec] */
    uint32_t     v_max;
    uint32_t     accel;
    uint32_t     decel;
} axis_status_t;

/* ------------------------------------------------------------------ */
/*  保持電流モード（全軸共通、DRV_ENが単一GPIOのため）                     */
/* ------------------------------------------------------------------ */
typedef enum {
    HOLD_MODE_NORMAL  = 0,   /* 既存動作: アイドルタイムアウトで無励磁 */
    HOLD_MODE_FULL    = 1,   /* アイドルタイムアウト無効化、常時フル励磁 */
    HOLD_MODE_REDUCED = 2,   /* アイドルタイムアウト無効化、全軸アイドル中は
                                 DRV_ENをPWMチョッピングし保持電流を低減 */
} hold_mode_t;

/* ------------------------------------------------------------------ */
/*  フォルト情報（GET FAULT_INFO 用）                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    fault_reason_t reason;
    uint8_t        axis_mask;      /* 影響軸ビットフィールド */
    int64_t        timestamp_us;   /* esp_timer_get_time() の値 */
} fault_info_t;

/* ------------------------------------------------------------------ */
/*  フォルトトレース（GET FAULT_TRACE 用、直近状態のリングバッファ）        */
/*  10ms間隔 × 400件 = 直近4秒分。エラー直前の挙動解析用。                 */
/*  サンプル間隔は固定のため、個々のタイムスタンプは持たない               */
/*  （クライアント側で「最新サンプル基準の何ms前か」を逆算する）。          */
/* ------------------------------------------------------------------ */
#define FAULT_TRACE_CAPACITY     400
#define FAULT_TRACE_INTERVAL_MS  10

typedef struct {
    uint8_t  state;        /* axis_state_t */
    int32_t  step_pos;
    int32_t  enc_steps;    /* エンコーダ換算位置 [microstep単位] */
    int32_t  diff;         /* enc_steps - step_pos（F-MOT-08 脱調判定値と同一） */
    int32_t  vel;          /* 符号付き速度 [steps/sec] */
    int16_t  current_mA;   /* 電流 [mA]（全軸共通センサ値、参考記録） */
} fault_trace_sample_t;

/* ------------------------------------------------------------------ */
/*  コールバック型                                                       */
/* ------------------------------------------------------------------ */
typedef void (*motor_event_cb_t)(uint8_t axis);
typedef void (*motor_fault_cb_t)(fault_reason_t reason, uint8_t axis_mask);

/* ------------------------------------------------------------------ */
/*  初期化                                                              */
/* ------------------------------------------------------------------ */
void motor_ctrl_init(void);

/* ------------------------------------------------------------------ */
/*  イベントコールバック登録                                              */
/* ------------------------------------------------------------------ */
void motor_register_move_done_cb(motor_event_cb_t cb);
void motor_register_move_aborted_cb(motor_event_cb_t cb);  /* EVT MOVE_ABORTED */
void motor_register_limit_hit_cb(motor_event_cb_t cb);     /* EVT LIMIT_HIT    */
void motor_register_fault_cb(motor_fault_cb_t cb);

/* ------------------------------------------------------------------ */
/*  有効化 / 無効化 (全軸共通 DRV_EN)                                   */
/* ------------------------------------------------------------------ */
void motor_enable(void);
void motor_disable(void);
bool motor_is_enabled(void);   /* 全軸共通 DRV_EN の状態 */

/* ------------------------------------------------------------------ */
/*  モーション制御                                                       */
/* ------------------------------------------------------------------ */
bool motor_move(uint8_t axis, int32_t steps);           /* 相対移動 */
bool motor_moveto(uint8_t axis, int32_t pos);           /* 絶対移動 */
bool motor_degrees_to_steps(uint8_t axis, float deg, int32_t *out_steps);
bool motor_vel(uint8_t axis, int32_t vel_signed);       /* 速度モード */
bool motor_stop(uint8_t axis);                          /* 台形減速停止 */
bool motor_stop_free(uint8_t axis);                     /* 台形減速後コイル解除 */
void motor_estop(fault_reason_t reason);                /* 全軸即時停止 + FAULT */

/* ------------------------------------------------------------------ */
/*  フォルト管理                                                         */
/* ------------------------------------------------------------------ */
bool motor_clear_fault(void);          /* true=成功, false=非FAULT状態 */
void motor_get_fault_info(fault_info_t *out);

/* 直近 FAULT_TRACE_CAPACITY 件を古い→新しい順に out へコピーする。実コピー件数を返す。 */
uint16_t motor_get_fault_trace(uint8_t axis, fault_trace_sample_t *out, uint16_t max_entries);

/* ------------------------------------------------------------------ */
/*  ステータス取得                                                       */
/* ------------------------------------------------------------------ */
bool motor_get_status(uint8_t axis, axis_status_t *out);
bool motor_is_moving(uint8_t axis);    /* ACCEL/CRUISE/DECEL/HOMING なら true */

/* ------------------------------------------------------------------ */
/*  パラメータ設定                                                       */
/* ------------------------------------------------------------------ */
bool motor_set_vmax(uint8_t axis, uint32_t vmax);
bool motor_set_accel(uint8_t axis, uint32_t accel_val);
bool motor_set_decel(uint8_t axis, uint32_t decel_val);
bool motor_set_idle_timeout(uint32_t timeout_ms);                        /* 全軸共通 */
bool motor_set_hold_mode(hold_mode_t mode);                               /* 全軸共通 */
hold_mode_t motor_get_hold_mode(void);
bool motor_set_hold_current_percent(uint8_t percent);                     /* 1〜100、全軸共通 */
uint8_t motor_get_hold_current_percent(void);
bool motor_set_soft_limit(uint8_t axis, int32_t min_p, int32_t max_p);  /* ソフトリミット */

/* ------------------------------------------------------------------ */
/*  Phase 1 テスト API (デバッグ用)                                     */
/* ------------------------------------------------------------------ */
bool motor_test_pulse(uint8_t axis, uint32_t freq_hz);
bool motor_stop_immediate(uint8_t axis);
bool motor_test_gpio_toggle(uint8_t axis, uint32_t count);

/* ------------------------------------------------------------------ */
/*  Phase 3: ホーミング / 脱調検出                                      */
/* ------------------------------------------------------------------ */
bool motor_home(uint8_t axis);
bool motor_set_stall_fault_th(uint8_t axis, uint32_t th);
bool motor_set_home_dir(uint8_t axis, int8_t dir);
bool motor_set_home_params(uint8_t axis, uint32_t v_coarse, uint32_t v_fine,
                            int32_t back_off, int32_t home_offset);
void motor_register_home_done_cb(motor_event_cb_t cb);
void motor_register_home_timeout_cb(motor_event_cb_t cb);

/* ------------------------------------------------------------------ */
/*  Phase 5: SYNC_MOVE (F-MOT-11)                                      */
/* ------------------------------------------------------------------ */
typedef void (*motor_sync_cb_t)(uint8_t axis_mask);
void motor_register_sync_done_cb(motor_sync_cb_t cb);
void motor_register_sync_aborted_cb(motor_sync_cb_t cb);
bool motor_sync_move(uint8_t n, const uint8_t *axes, const int32_t *steps);

/* ------------------------------------------------------------------ */
/*  追加ゲッター（NVS 永続化・状態確認用）                               */
/* ------------------------------------------------------------------ */
uint32_t motor_get_stall_fault_th(uint8_t axis);
bool     motor_get_soft_limit(uint8_t axis, int32_t *out_min, int32_t *out_max);
int8_t   motor_get_enc_dir(uint8_t axis);
bool     motor_set_enc_dir(uint8_t axis, int8_t dir);   /* +1=正 / -1=逆接続 */
motor_type_t motor_get_motor_type(uint8_t axis);
bool         motor_set_motor_type(uint8_t axis, motor_type_t type);
uint16_t motor_get_mpc_x100(uint8_t axis);
bool     motor_set_mpc_x100(uint8_t axis, uint16_t mpc); /* microsteps_per_count×100 */
uint32_t motor_get_v_home_coarse(uint8_t axis);
uint32_t motor_get_v_home_fine(uint8_t axis);
int32_t  motor_get_back_off_steps(uint8_t axis);
int32_t  motor_get_home_offset_steps(uint8_t axis);
int8_t   motor_get_home_dir(uint8_t axis);
