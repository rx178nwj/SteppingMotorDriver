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

typedef struct {
    axis_state_t state;
    int32_t      pos;          /* 現在位置 [steps] */
    int32_t      vel;          /* 符号付き速度 [steps/sec] */
    uint32_t     v_max;
    uint32_t     accel;
    uint32_t     decel;
} axis_status_t;

/* ------------------------------------------------------------------ */
/*  フォルト情報（GET FAULT_INFO 用）                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    fault_reason_t reason;
    uint8_t        axis_mask;      /* 影響軸ビットフィールド */
    int64_t        timestamp_us;   /* esp_timer_get_time() の値 */
} fault_info_t;

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
void motor_register_fault_cb(motor_fault_cb_t cb);

/* ------------------------------------------------------------------ */
/*  有効化 / 無効化 (全軸共通 DRV_EN)                                   */
/* ------------------------------------------------------------------ */
void motor_enable(void);
void motor_disable(void);

/* ------------------------------------------------------------------ */
/*  モーション制御                                                       */
/* ------------------------------------------------------------------ */
bool motor_move(uint8_t axis, int32_t steps);           /* 相対移動 */
bool motor_moveto(uint8_t axis, int32_t pos);           /* 絶対移動 */
bool motor_vel(uint8_t axis, int32_t vel_signed);       /* 速度モード */
bool motor_stop(uint8_t axis);                          /* 台形減速停止 */
bool motor_stop_free(uint8_t axis);                     /* 台形減速後コイル解除 */
void motor_estop(fault_reason_t reason);                /* 全軸即時停止 + FAULT */

/* ------------------------------------------------------------------ */
/*  フォルト管理                                                         */
/* ------------------------------------------------------------------ */
bool motor_clear_fault(void);          /* true=成功, false=非FAULT状態 */
void motor_get_fault_info(fault_info_t *out);

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
bool motor_set_idle_timeout(uint32_t timeout_ms);       /* 全軸共通 */

/* ------------------------------------------------------------------ */
/*  Phase 1 テスト API (デバッグ用)                                     */
/* ------------------------------------------------------------------ */
bool motor_test_pulse(uint8_t axis, uint32_t freq_hz);
bool motor_stop_immediate(uint8_t axis);
bool motor_test_gpio_toggle(uint8_t axis, uint32_t count);
