#include "telemetry.h"

#include "adc_monitor.h"
#include "config.h"
#include "encoder.h"
#include "error_log.h"
#include "gear_monitor.h"
#include "motor_ctrl.h"

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_mac.h"

static char s_board_id[13];

static const char *axis_state_name(axis_state_t state)
{
    switch (state) {
    case AXIS_SLEEP:  return "SLEEP";
    case AXIS_IDLE:   return "IDLE";
    case AXIS_ACCEL:  return "ACCEL";
    case AXIS_CRUISE: return "CRUISE";
    case AXIS_DECEL:  return "DECEL";
    case AXIS_HOMING: return "HOMING";
    case AXIS_FAULT:  return "FAULT";
    default:          return "UNKNOWN";
    }
}

static const char *fault_name(fault_reason_t reason)
{
    switch (reason) {
    case FAULT_ESTOP:       return "ESTOP";
    case FAULT_OVERCURRENT: return "OVERCURRENT";
    case FAULT_STALL:       return "STALL";
    default:                return "NONE";
    }
}

static bool appendf(char *buf, size_t size, size_t *used, const char *fmt, ...)
{
    if (*used >= size) return false;
    va_list ap;
    va_start(ap, fmt);
    int written = vsnprintf(buf + *used, size - *used, fmt, ap);
    va_end(ap);
    if (written < 0 || (size_t)written >= size - *used) {
        buf[size - 1] = '\0';
        return false;
    }
    *used += (size_t)written;
    return true;
}

const char *telemetry_board_id(void)
{
    if (s_board_id[0] == '\0') {
        uint8_t mac[6] = {0};
        if (esp_read_mac(mac, ESP_MAC_BASE) != ESP_OK) {
            memset(mac, 0, sizeof(mac));
        }
        snprintf(s_board_id, sizeof(s_board_id),
                 "%02X%02X%02X%02X%02X%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    return s_board_id;
}

bool telemetry_format_device_info(char *buf, size_t size)
{
    const esp_app_desc_t *app = esp_app_get_description();
    int n = snprintf(buf, size,
                     "{\"product\":\"stepping_motor_driver\","
                     "\"board_id\":\"%s\",\"protocol_version\":1,"
                     "\"fw_version\":\"%s\"}",
                     telemetry_board_id(), app ? app->version : "unknown");
    return n >= 0 && (size_t)n < size;
}

bool telemetry_format_axis_status(char *buf, size_t size)
{
    size_t used = 0;
    if (!appendf(buf, size, &used, "[")) return false;
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        axis_status_t status;
        if (!motor_get_status(axis, &status)) return false;
        if (!appendf(buf, size, &used,
                     "%s{\"axis\":%u,\"state\":\"%s\",\"pos\":%ld,"
                     "\"vel\":%ld,\"enc\":%ld}",
                     axis == 0 ? "" : ",", (unsigned)axis,
                     axis_state_name(status.state), (long)status.pos,
                     (long)status.vel, (long)encoder_get_pos(axis))) {
            return false;
        }
    }
    return appendf(buf, size, &used, "]");
}

bool telemetry_format_power(char *buf, size_t size)
{
    int n = snprintf(buf, size,
                     "{\"pot\":[%d,%d,%d],\"current_mA\":%.1f,"
                     "\"voltage_mV\":%.0f}",
                     adc_get_raw_mv(0), adc_get_raw_mv(1), adc_get_raw_mv(2),
                     (double)adc_get_current_mA(),
                     (double)(adc_get_voltage_V() * 1000.0f));
    return n >= 0 && (size_t)n < size;
}

bool telemetry_format_fault(char *buf, size_t size)
{
    fault_info_t fault;
    motor_get_fault_info(&fault);
    int n = snprintf(buf, size,
                     "{\"reason\":\"%s\",\"axis_mask\":%u,"
                     "\"timestamp_us\":%lld}",
                     fault_name(fault.reason), (unsigned)fault.axis_mask,
                     (long long)fault.timestamp_us);
    return n >= 0 && (size_t)n < size;
}

bool telemetry_format_error_log_latest(char *buf, size_t size)
{
    error_log_entry_t entry;
    if (!error_log_get_latest(&entry)) {
        int n = snprintf(buf, size, "{\"seq\":0,\"t\":0,\"code\":\"NONE\",\"msg\":\"\"}");
        return n >= 0 && (size_t)n < size;
    }
    int n = snprintf(buf, size,
                     "{\"seq\":%lu,\"t\":%lld,\"code\":\"%s\",\"msg\":\"%s\"}",
                     (unsigned long)entry.seq, (long long)entry.timestamp_us,
                     entry.code, entry.message);
    return n >= 0 && (size_t)n < size;
}

bool telemetry_format_gear_angle(char *buf, size_t size)
{
    if (gear_monitor_get_state() != GEAR_STATE_READY) {
        int n = snprintf(buf, size, "{\"state\":\"UNAVAILABLE\"}");
        return n >= 0 && (size_t)n < size;
    }

    size_t used = 0;
    if (!appendf(buf, size, &used, "[")) return false;
    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        gear_axis_status_t gear;
        bool available = gear_monitor_get_axis_status(axis, &gear) &&
                         gear.enabled && gear.ok;
        if (!appendf(buf, size, &used,
                     "%s{\"axis\":%u,\"angle_deg\":%.2f,\"state\":\"%s\"}",
                     axis == 0 ? "" : ",", (unsigned)axis,
                     available ? (double)gear.angle_deg : 0.0,
                     available ? "OK" : "UNAVAILABLE")) {
            return false;
        }
    }
    return appendf(buf, size, &used, "]");
}

bool telemetry_format_joint_angle(char *buf, size_t size)
{
    size_t used = 0;
    if (!appendf(buf, size, &used, "[")) return false;

    for (uint8_t axis = 0; axis < NUM_AXES; axis++) {
        axis_status_t status;
        if (!motor_get_status(axis, &status)) return false;

        uint32_t steps_per_rev = config_get_steps_per_rev(axis);
        uint32_t encoder_ppr = config_get_encoder_ppr(axis);
        float gear_ratio = config_get_gear_ratio(axis);
        float pot_scale = config_get_pot_scale_deg(axis);
        int32_t pot_zero = config_get_pot_zero_offset(axis);
        int pot_raw = adc_get_raw_count(axis);
        if (steps_per_rev == 0 || encoder_ppr == 0 ||
            !isfinite(gear_ratio) || gear_ratio <= 0.0f ||
            !isfinite(pot_scale) || pot_scale <= 0.0f || pot_raw < 0) {
            return false;
        }

        double pos_deg = (double)status.pos * 360.0 /
                         ((double)steps_per_rev * (double)gear_ratio);
        double enc_deg = (double)encoder_get_pos(axis) * 360.0 /
                         ((double)encoder_ppr * 4.0 * (double)gear_ratio);
        double pot_deg_raw = (double)pot_raw * (double)pot_scale;
        double pot_deg_zeroed = ((double)pot_raw - (double)pot_zero) *
                                (double)pot_scale;
        if (!appendf(buf, size, &used,
                     "%s{\"axis\":%u,\"pos_deg\":%.3f,\"enc_deg\":%.3f,"
                     "\"pot_deg_raw\":%.3f,\"pot_deg_zeroed\":%.3f}",
                     axis == 0 ? "" : ",", (unsigned)axis,
                     pos_deg, enc_deg, pot_deg_raw, pot_deg_zeroed)) {
            return false;
        }
    }
    return appendf(buf, size, &used, "]");
}
