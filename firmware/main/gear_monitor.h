#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "gpio_config.h"

typedef enum {
    GEAR_STATE_INIT = 0,
    GEAR_STATE_READY,
    GEAR_STATE_UNAVAILABLE,
    GEAR_STATE_VERSION_MISMATCH,
} gear_state_t;

typedef struct {
    bool     enabled;
    bool     ok;
    bool     boot_angle_valid;
    bool     home_angle_valid;
    float    raw_angle_deg;
    float    angle_deg;
    float    motor_equiv_deg;
    float    boot_angle_deg;
    float    home_angle_deg;
    float    deviation_deg;
    uint16_t raw_count;
    uint8_t  sample_lo;
} gear_axis_status_t;

typedef enum {
    GEAR_EVENT_DEGRADED = 0,
    GEAR_EVENT_RECOVERED,
    GEAR_EVENT_UNAVAILABLE,
    GEAR_EVENT_AVAILABLE,
    GEAR_EVENT_DEVIATION_WARN,
} gear_event_t;

typedef void (*gear_event_cb_t)(gear_event_t event, uint8_t axis, float value);

/* I2C と GearMonitorTask を初期化する。失敗してもモーター制御には影響しない。 */
void gear_monitor_init(void);

/* CommTask などの通知先を登録する。 */
void gear_monitor_register_event_callback(gear_event_cb_t cb);

gear_state_t gear_monitor_get_state(void);
const char  *gear_monitor_state_name(gear_state_t state);
bool         gear_monitor_get_axis_status(uint8_t axis, gear_axis_status_t *out);

/* HOME_DONE 時点の角度とモーター位置を基準として記録する。 */
void gear_monitor_mark_home(uint8_t axis);

