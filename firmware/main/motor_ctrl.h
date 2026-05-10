#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  軸状態                                                              */
/* ------------------------------------------------------------------ */
typedef enum {
    AXIS_SLEEP = 0,
    AXIS_IDLE,
    AXIS_ACCEL,
    AXIS_CRUISE,
    AXIS_DECEL,
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
/*  初期化                                                              */
/* ------------------------------------------------------------------ */
void motor_ctrl_init(void);

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
void motor_estop(void);                                 /* 全軸即時停止 + FAULT */

/* ------------------------------------------------------------------ */
/*  フォルト管理                                                         */
/* ------------------------------------------------------------------ */
void motor_clear_fault(void);

/* ------------------------------------------------------------------ */
/*  ステータス取得                                                       */
/* ------------------------------------------------------------------ */
bool motor_get_status(uint8_t axis, axis_status_t *out);

/* ------------------------------------------------------------------ */
/*  パラメータ設定                                                       */
/* ------------------------------------------------------------------ */
bool motor_set_vmax(uint8_t axis, uint32_t vmax);
bool motor_set_accel(uint8_t axis, uint32_t accel_val);
bool motor_set_decel(uint8_t axis, uint32_t decel_val);

/* ------------------------------------------------------------------ */
/*  Phase 1 テスト API (デバッグ用)                                     */
/* ------------------------------------------------------------------ */
bool motor_test_pulse(uint8_t axis, uint32_t freq_hz);
bool motor_stop_immediate(uint8_t axis);
bool motor_test_gpio_toggle(uint8_t axis, uint32_t count);
