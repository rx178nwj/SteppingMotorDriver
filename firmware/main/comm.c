#include "comm.h"
#include "error_log.h"
#include "motor_ctrl.h"
#include "encoder.h"
#include "adc_monitor.h"
#include "config.h"
#include "gear_monitor.h"
#include "ble_telemetry.h"
#include "wifi_telemetry.h"
#include "telemetry.h"
#include "status_led.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "comm";

static SemaphoreHandle_t s_tx_mutex;

/* 通信ウォッチドッグ: 最終受信時刻 [µs] */
static volatile int64_t  s_last_rx_us   = 0;
static          uint32_t s_wdog_ms      = 5000;  /* 0 = 無効 */
static          bool     s_wdog_enabled = true;

/* ------------------------------------------------------------------ */
/*  ヘルパー                                                            */
/* ------------------------------------------------------------------ */
static const char *state_name(axis_state_t s)
{
    switch (s) {
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

static const char *fault_reason_name(fault_reason_t r)
{
    switch (r) {
    case FAULT_ESTOP:       return "ESTOP";
    case FAULT_OVERCURRENT: return "OVERCURRENT";
    case FAULT_STALL:       return "STALL";
    default:                return "NONE";
    }
}

/* 単軸モーションコマンド (MOVE/MOVETO/MOVE_DEG/MOVETO_DEG) の開始前 FAULT 確認用 */
static bool axis_in_fault(uint8_t axis)
{
    axis_status_t st;
    motor_get_status(axis, &st);
    return st.state == AXIS_FAULT;
}

/* ------------------------------------------------------------------ */
/*  イベントコールバック（motor_ctrl → comm）                            */
/* ------------------------------------------------------------------ */
static void on_move_done(uint8_t axis)
{
    comm_sendf("EVT MOVE_DONE %u\n", (unsigned)axis);
}

static void on_move_aborted(uint8_t axis)
{
    comm_sendf("EVT MOVE_ABORTED %u\n", (unsigned)axis);
}

static void on_limit_hit(uint8_t axis)
{
    comm_sendf("EVT LIMIT_HIT %u\n", (unsigned)axis);
}

static void on_fault(fault_reason_t reason, uint8_t axis_mask)
{
    comm_sendf("EVT FAULT %s 0x%02x\n",
               fault_reason_name(reason), (unsigned)axis_mask);
}

static void on_home_done(uint8_t axis)
{
    gear_monitor_mark_home(axis);
    comm_sendf("EVT HOME_DONE %u\n", (unsigned)axis);
}

static void on_home_timeout(uint8_t axis)
{
    comm_sendf("EVT HOME_TIMEOUT %u\n", (unsigned)axis);
}

static void on_overcurrent(float current_mA)
{
    /* EVT OVERCURRENT <mA>: 過電流検出（motor_estop は adc_monitor 内で発火） */
    comm_sendf("EVT OVERCURRENT ALL %.1f\n", (double)current_mA);
}

static void on_sync_done(uint8_t axis_mask)
{
    comm_sendf("EVT SYNC_DONE 0x%02x\n", (unsigned)axis_mask);
}

static void on_sync_aborted(uint8_t axis_mask)
{
    comm_sendf("EVT SYNC_ABORTED 0x%02x\n", (unsigned)axis_mask);
}

static void on_gear_event(gear_event_t event, uint8_t axis, float value)
{
    switch (event) {
    case GEAR_EVENT_DEGRADED:
        comm_sendf("EVT GEAR_DEGRADED %u\n", (unsigned)axis);
        break;
    case GEAR_EVENT_RECOVERED:
        comm_sendf("EVT GEAR_RECOVERED %u\n", (unsigned)axis);
        break;
    case GEAR_EVENT_UNAVAILABLE:
        comm_send("EVT GEAR_UNAVAILABLE\n");
        break;
    case GEAR_EVENT_AVAILABLE:
        comm_send("EVT GEAR_AVAILABLE\n");
        break;
    case GEAR_EVENT_DEVIATION_WARN:
        comm_sendf("EVT GEAR_DEVIATION_WARN %u %.2f\n",
                   (unsigned)axis, (double)value);
        break;
    }
}

static void send_gear_unavailable_error(void)
{
    if (gear_monitor_get_state() == GEAR_STATE_VERSION_MISMATCH) {
        comm_send("ERR E014 GEAR_VERSION_MISMATCH\n");
    } else {
        comm_send("ERR E013 GEAR_UNAVAILABLE\n");
    }
}

static const char *command_args(const char *line, const char *command)
{
    size_t len = strlen(command);
    if (strncmp(line, command, len) != 0 ||
        (line[len] != '\0' && line[len] != ' ')) {
        return NULL;
    }
    const char *args = line + len;
    while (*args == ' ') args++;
    return args;
}

static bool parse_axis_arg(const char *args, long *axis)
{
    if (!args || *args == '\0') return false;
    char *end;
    long value = strtol(args, &end, 10);
    if (end == args) return false;
    while (*end == ' ') end++;
    if (*end != '\0') return false;
    *axis = value;
    return true;
}

static bool parse_axis_float_args(const char *args, long *axis, float *value)
{
    if (!args || *args == '\0') return false;
    char *end;
    long parsed_axis = strtol(args, &end, 10);
    if (end == args || *end != ' ') return false;
    args = end;
    while (*args == ' ') args++;
    if (*args == '\0') return false;
    float parsed_value = strtof(args, &end);
    if (end == args) return false;
    while (*end == ' ') end++;
    if (*end != '\0') return false;
    *axis = parsed_axis;
    *value = parsed_value;
    return true;
}

/* ------------------------------------------------------------------ */
/*  StatusTask — 100ms ハートビート・内部ログ出力 (F-COM-03)            */
/* ------------------------------------------------------------------ */
static volatile bool s_heartbeat_enabled = false;

static void status_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (s_heartbeat_enabled) {
            comm_sendf("HB %lld", (long long)(esp_timer_get_time() / 1000LL));
            for (uint8_t i = 0; i < NUM_AXES; i++) {
                axis_status_t st;
                motor_get_status(i, &st);
                const char *sname;
                switch (st.state) {
                case AXIS_SLEEP:  sname = "SL"; break;
                case AXIS_IDLE:   sname = "ID"; break;
                case AXIS_ACCEL:  sname = "AC"; break;
                case AXIS_CRUISE: sname = "CR"; break;
                case AXIS_DECEL:  sname = "DE"; break;
                case AXIS_HOMING: sname = "HO"; break;
                case AXIS_FAULT:  sname = "FA"; break;
                default:          sname = "??"; break;
                }
                comm_sendf(" %s/%ld", sname, (long)st.pos);
            }
            comm_send("\n");
        }
    }
}

/* ------------------------------------------------------------------ */
/*  WatchdogTask — 無通信タイムアウト監視                               */
/* ------------------------------------------------------------------ */
static void watchdog_task(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));

        if (!s_wdog_enabled || s_wdog_ms == 0) continue;
        if (s_last_rx_us == 0) continue;   /* 1回も受信していない */

        int64_t elapsed_ms = (esp_timer_get_time() - s_last_rx_us) / 1000LL;
        if (elapsed_ms >= (int64_t)s_wdog_ms) {
            ESP_LOGW(TAG, "COMM_TIMEOUT: %lld ms, stopping all axes", elapsed_ms);
            for (uint8_t i = 0; i < NUM_AXES; i++) motor_stop(i);
            s_last_rx_us = 0;
            comm_send("EVT COMM_TIMEOUT\n");
        }
    }
}

/* ------------------------------------------------------------------ */
/*  コマンド dispatch                                                   */
/* ------------------------------------------------------------------ */
static void dispatch(const char *line)
{
    /* ---------- 疎通確認 ---------- */
    if (strcmp(line, "PING") == 0) {
        comm_send("OK PONG\n");
        return;
    }
    if (strcmp(line, "IDENTITY") == 0) {
        comm_sendf("OK {\"product\":\"stepping_motor_driver\","
                   "\"board_id\":\"%s\",\"protocol_version\":1}\n",
                   telemetry_board_id());
        return;
    }

    unsigned axis;

    /* ---------- SYNC_MOVE (F-MOT-11) ---------- */
    if (strncmp(line, "SYNC_MOVE", 9) == 0 && (line[9] == ' ' || line[9] == '\0')) {
        const char *p   = line + 9;
        char       *end;

        /* n: 同期軸数 */
        long n_val = strtol(p, &end, 10);
        if (end == p || n_val < 2 || n_val > (long)NUM_AXES) {
            comm_send("ERR E002 INVALID_ARG\n"); return;
        }
        unsigned n = (unsigned)n_val;
        p = end;

        uint8_t  sync_axes[NUM_AXES];
        int32_t  sync_steps[NUM_AXES];
        uint8_t  used_mask = 0;

        for (unsigned k = 0; k < n; k++) {
            /* axis */
            long ax_v = strtol(p, &end, 10);
            if (end == p)                       { comm_send("ERR E002 INVALID_ARG\n");  return; }
            if (ax_v < 0 || ax_v >= NUM_AXES)   { comm_send("ERR E003 INVALID_AXIS\n"); return; }
            p = end;
            /* steps */
            long st_v = strtol(p, &end, 10);
            if (end == p)                       { comm_send("ERR E002 INVALID_ARG\n");  return; }
            p = end;

            uint8_t axb = (uint8_t)ax_v;
            if (used_mask & (1u << axb))        { comm_send("ERR E011 DUPLICATE_AXIS\n"); return; }
            used_mask |= (1u << axb);

            if (motor_is_moving(axb))           { comm_send("ERR E008 MOTION_IN_PROGRESS\n"); return; }
            axis_status_t chk;
            motor_get_status(axb, &chk);
            if (chk.state == AXIS_FAULT)        { comm_send("ERR E005 FAULT\n"); return; }

            sync_axes[k]  = axb;
            sync_steps[k] = (int32_t)st_v;
        }

        if (!motor_sync_move(n, sync_axes, sync_steps)) {
            comm_send("ERR E006 SOFT_LIMIT\n");
        } else {
            comm_send("OK\n");
        }
        return;
    }

    /* ---------- ハートビート切替 ---------- */
    if (strcmp(line, "HEARTBEAT ON") == 0) {
        s_heartbeat_enabled = true;
        comm_send("OK\n");
        return;
    }
    if (strcmp(line, "HEARTBEAT OFF") == 0) {
        s_heartbeat_enabled = false;
        comm_send("OK\n");
        return;
    }

    /* ---------- 有効化 / 無効化 ---------- */
    if (strcmp(line, "ENABLE") == 0 ||
        strncmp(line, "ENABLE ", 7) == 0) {
        motor_enable();
        comm_send("OK\n");
        return;
    }
    if (strcmp(line, "DISABLE") == 0 ||
        strncmp(line, "DISABLE ", 8) == 0) {
        motor_disable();
        comm_send("OK\n");
        return;
    }

    /* ---------- モーション ---------- */
    const char *args = command_args(line, "MOVE_DEG");
    if (args) {
        long parsed_axis;
        float deg;
        int32_t converted;
        if (!parse_axis_float_args(args, &parsed_axis, &deg)) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (parsed_axis < 0 || parsed_axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (!motor_degrees_to_steps((uint8_t)parsed_axis, deg, &converted)) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (motor_is_moving((uint8_t)parsed_axis)) {
            comm_send("ERR E008 MOTION_IN_PROGRESS\n");
        } else if (axis_in_fault((uint8_t)parsed_axis)) {
            comm_send("ERR E005 FAULT\n");
        } else if (!motor_is_enabled()) {
            comm_send("ERR E009 NOT_ENABLED\n");
        } else {
            comm_send(motor_move((uint8_t)parsed_axis, converted)
                      ? "OK\n" : "ERR E006 SOFT_LIMIT\n");
        }
        return;
    }

    args = command_args(line, "MOVETO_DEG");
    if (args) {
        long parsed_axis;
        float deg;
        int32_t converted;
        if (!parse_axis_float_args(args, &parsed_axis, &deg)) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (parsed_axis < 0 || parsed_axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (!motor_degrees_to_steps((uint8_t)parsed_axis, deg, &converted)) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (motor_is_moving((uint8_t)parsed_axis)) {
            comm_send("ERR E008 MOTION_IN_PROGRESS\n");
        } else if (axis_in_fault((uint8_t)parsed_axis)) {
            comm_send("ERR E005 FAULT\n");
        } else if (!motor_is_enabled()) {
            comm_send("ERR E009 NOT_ENABLED\n");
        } else {
            comm_send(motor_moveto((uint8_t)parsed_axis, converted)
                      ? "OK\n" : "ERR E006 SOFT_LIMIT\n");
        }
        return;
    }

    int steps;
    if (sscanf(line, "MOVE %u %d", &axis, &steps) == 2) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (motor_is_moving((uint8_t)axis)) {
            comm_send("ERR E008 MOTION_IN_PROGRESS\n");
        } else if (axis_in_fault((uint8_t)axis)) {
            comm_send("ERR E005 FAULT\n");
        } else if (!motor_is_enabled()) {
            comm_send("ERR E009 NOT_ENABLED\n");
        } else {
            comm_send(motor_move((uint8_t)axis, steps) ? "OK\n" : "ERR E006 SOFT_LIMIT\n");
        }
        return;
    }

    int pos;
    if (sscanf(line, "MOVETO %u %d", &axis, &pos) == 2) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (motor_is_moving((uint8_t)axis)) {
            comm_send("ERR E008 MOTION_IN_PROGRESS\n");
        } else if (axis_in_fault((uint8_t)axis)) {
            comm_send("ERR E005 FAULT\n");
        } else if (!motor_is_enabled()) {
            comm_send("ERR E009 NOT_ENABLED\n");
        } else {
            comm_send(motor_moveto((uint8_t)axis, pos) ? "OK\n" : "ERR E006 SOFT_LIMIT\n");
        }
        return;
    }

    int vel;
    if (sscanf(line, "VEL %u %d", &axis, &vel) == 2) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_send(motor_vel((uint8_t)axis, vel) ? "OK\n" : "ERR E005 FAULT\n");
        }
        return;
    }

    /* ---------- 停止 ---------- */
    if (sscanf(line, "STOP %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            motor_stop((uint8_t)axis);
            comm_send("OK\n");
        }
        return;
    }
    if (strcmp(line, "STOP ALL") == 0) {
        for (uint8_t i = 0; i < NUM_AXES; i++) motor_stop(i);
        comm_send("OK\n");
        return;
    }

    /* STOP_FREE: 台形減速後にコイル解除 */
    if (sscanf(line, "STOP_FREE %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            motor_stop_free((uint8_t)axis);
            comm_send("OK\n");
        }
        return;
    }
    if (strcmp(line, "STOP_FREE ALL") == 0) {
        for (uint8_t i = 0; i < NUM_AXES; i++) motor_stop_free(i);
        comm_send("OK\n");
        return;
    }

    if (strcmp(line, "ESTOP") == 0) {
        motor_estop(FAULT_ESTOP);
        comm_send("OK\n");
        return;
    }

    /* ---------- ホーミング ---------- */
    if (sscanf(line, "HOME %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            axis_status_t home_st;
            motor_get_status((uint8_t)axis, &home_st);
            if (motor_is_moving((uint8_t)axis) || home_st.state == AXIS_FAULT) {
                comm_send("ERR E008 MOTION_IN_PROGRESS\n");
            } else if (!motor_home((uint8_t)axis)) {
                comm_send("ERR E005 FAULT\n");
            } else {
                comm_send("OK\n");
            }
        }
        return;
    }

    /* ---------- フォルト ---------- */
    if (strcmp(line, "CLEAR_FAULT") == 0) {
        if (motor_clear_fault()) {
            comm_send("OK\n");
        } else {
            comm_send("ERR E010 NOT_IN_FAULT\n");
        }
        return;
    }
    if (strcmp(line, "RECOVER") == 0) {
        /* CLEAR_FAULT (FAULT状態でなければ no-op) → ENABLE を1コマンドにまとめた復帰用ショートカット。 */
        motor_clear_fault();
        motor_enable();
        comm_send("OK\n");
        return;
    }

    /* ---------- GET ---------- */
    args = command_args(line, "GET POS_DEG");
    if (args) {
        long parsed_axis;
        axis_status_t st;
        if (!parse_axis_arg(args, &parsed_axis)) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (parsed_axis < 0 || parsed_axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (!motor_get_status((uint8_t)parsed_axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            uint32_t steps_per_rev = config_get_steps_per_rev((uint8_t)parsed_axis);
            float gear_ratio = config_get_gear_ratio((uint8_t)parsed_axis);
            if (steps_per_rev == 0 || !isfinite(gear_ratio) || gear_ratio <= 0.0f) {
                comm_send("ERR E002 INVALID_PARAM\n");
            } else {
                double deg = (double)st.pos * 360.0 /
                             ((double)steps_per_rev * (double)gear_ratio);
                comm_sendf("OK %.3f\n", deg);
            }
        }
        return;
    }

    args = command_args(line, "GET ENC_DEG");
    if (args) {
        long parsed_axis;
        if (!parse_axis_arg(args, &parsed_axis)) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (parsed_axis < 0 || parsed_axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (motor_get_motor_type((uint8_t)parsed_axis) == MOTOR_TYPE_OPEN_LOOP) {
            comm_send("ERR E002 INVALID_PARAM\n");   /* OPEN_LOOP軸: エンコーダ未装備 */
        } else {
            uint32_t encoder_ppr = config_get_encoder_ppr((uint8_t)parsed_axis);
            float gear_ratio = config_get_gear_ratio((uint8_t)parsed_axis);
            if (encoder_ppr == 0 || !isfinite(gear_ratio) || gear_ratio <= 0.0f) {
                comm_send("ERR E002 INVALID_PARAM\n");
            } else {
                double deg = (double)encoder_get_pos((uint8_t)parsed_axis) * 360.0 /
                             ((double)encoder_ppr * 4.0 * (double)gear_ratio);
                comm_sendf("OK %.3f\n", deg);
            }
        }
        return;
    }

    args = command_args(line, "GET POT_DEG");
    if (args) {
        long parsed_axis;
        if (!parse_axis_arg(args, &parsed_axis)) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (parsed_axis < 0 || parsed_axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            int raw = adc_get_raw_count((uint8_t)parsed_axis);
            float scale = config_get_pot_scale_deg((uint8_t)parsed_axis);
            int32_t zero = config_get_pot_zero_offset((uint8_t)parsed_axis);
            double raw_deg = (double)raw * (double)scale;
            double zeroed_deg = ((double)raw - (double)zero) * (double)scale;
            comm_sendf("OK %.3f %.3f\n", raw_deg, zeroed_deg);
        }
        return;
    }

    if (sscanf(line, "GET STATE %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %s\n", state_name(st.state));
        }
        return;
    }
    if (sscanf(line, "GET POS %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %ld\n", (long)st.pos);
        }
        return;
    }
    if (sscanf(line, "GET VEL %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %ld\n", (long)st.vel);
        }
        return;
    }
    if (sscanf(line, "GET ENC %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (motor_get_motor_type((uint8_t)axis) == MOTOR_TYPE_OPEN_LOOP) {
            comm_send("ERR E002 INVALID_PARAM\n");   /* OPEN_LOOP軸: エンコーダ未装備 */
        } else {
            comm_sendf("OK %ld\n", (long)encoder_get_pos((uint8_t)axis));
        }
        return;
    }
    if (sscanf(line, "GET VMAX %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %lu\n", (unsigned long)st.v_max);
        }
        return;
    }
    if (sscanf(line, "GET ACCEL %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %lu\n", (unsigned long)st.accel);
        }
        return;
    }
    if (sscanf(line, "GET DECEL %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %lu\n", (unsigned long)st.decel);
        }
        return;
    }
    if (sscanf(line, "GET GEAR_RATIO %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %.4f\n", (double)config_get_gear_ratio((uint8_t)axis));
        }
        return;
    }
    if (sscanf(line, "GET MOTOR_TYPE %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %u\n", (unsigned)motor_get_motor_type((uint8_t)axis));
        }
        return;
    }
    if (sscanf(line, "GET DRIVER_TYPE %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %u\n", (unsigned)motor_get_driver_type((uint8_t)axis));
        }
        return;
    }

    if (sscanf(line, "GET GEAR_ANGLE %u", &axis) == 1) {
        gear_axis_status_t gear;
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (gear_monitor_get_state() != GEAR_STATE_READY ||
                   !gear_monitor_get_axis_status((uint8_t)axis, &gear) ||
                   !gear.enabled || !gear.ok) {
            send_gear_unavailable_error();
        } else {
            comm_sendf("OK %.2f\n", (double)gear.angle_deg);
        }
        return;
    }
    if (sscanf(line, "GET GEAR_RAW %u", &axis) == 1) {
        gear_axis_status_t gear;
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (gear_monitor_get_state() != GEAR_STATE_READY ||
                   !gear_monitor_get_axis_status((uint8_t)axis, &gear) ||
                   !gear.enabled || !gear.ok) {
            send_gear_unavailable_error();
        } else {
            comm_sendf("OK %.2f\n", (double)gear.raw_angle_deg);
        }
        return;
    }
    if (strcmp(line, "GET GEAR_STATUS") == 0) {
        gear_state_t gear_state = gear_monitor_get_state();
        comm_sendf("OK {\"bridge\":\"%s\",\"axes\":[",
                   gear_monitor_state_name(gear_state));
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            gear_axis_status_t gear;
            gear_monitor_get_axis_status(i, &gear);
            comm_sendf("%s{\"ok\":%s,\"enabled\":%s,\"deg\":%.2f,"
                       "\"raw\":%.2f,\"motor_equiv_deg\":%.2f,"
                       "\"boot_valid\":%s,\"boot_deg\":%.2f,"
                       "\"home_valid\":%s,\"home_deg\":%.2f,"
                       "\"deviation\":%.2f,\"sample\":%u}",
                       i == 0 ? "" : ",",
                       gear.ok ? "true" : "false",
                       gear.enabled ? "true" : "false",
                       (double)gear.angle_deg,
                       (double)gear.raw_angle_deg,
                       (double)gear.motor_equiv_deg,
                       gear.boot_angle_valid ? "true" : "false",
                       (double)gear.boot_angle_deg,
                       gear.home_angle_valid ? "true" : "false",
                       (double)gear.home_angle_deg,
                       (double)gear.deviation_deg,
                       (unsigned)gear.sample_lo);
        }
        comm_send("]}\n");
        return;
    }
    if (sscanf(line, "GET GEAR_DEVIATION %u", &axis) == 1) {
        gear_axis_status_t gear;
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (gear_monitor_get_state() != GEAR_STATE_READY ||
                   !gear_monitor_get_axis_status((uint8_t)axis, &gear) ||
                   !gear.ok || !gear.home_angle_valid) {
            send_gear_unavailable_error();
        } else {
            comm_sendf("OK %.2f\n", (double)gear.deviation_deg);
        }
        return;
    }

    /* GET ADC <ch>: ch=0〜2→生 mV / ch=3→電源電圧 V / ch=4→電流 mA */
    if (sscanf(line, "GET ADC %u", &axis) == 1) {
        if (axis > 4) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (axis == 4) {
            comm_sendf("OK %.1f\n", (double)adc_get_current_mA());
        } else if (axis == 3) {
            comm_sendf("OK %.3f\n", (double)adc_get_voltage_V());
        } else {
            comm_sendf("OK %d\n", adc_get_raw_mv((uint8_t)axis));
        }
        return;
    }

    /* GET FAULT_INFO (F-MOT-07c) */
    if (strcmp(line, "GET FAULT_INFO") == 0) {
        fault_info_t fi;
        motor_get_fault_info(&fi);
        comm_sendf("OK %s 0x%02x %lld\n",
                   fault_reason_name(fi.reason),
                   (unsigned)fi.axis_mask,
                   (long long)fi.timestamp_us);
        return;
    }
    if (strcmp(line, "GET LOG") == 0) {
        static error_log_entry_t entries[ERROR_LOG_CAPACITY];   /* ≈1.9KB、CommTaskスタック節約のためBSSに配置 */
        uint32_t n = error_log_dump(entries, ERROR_LOG_CAPACITY);
        comm_send("OK [");
        for (uint32_t i = 0; i < n; i++) {
            comm_sendf("%s{\"t\":%lld,\"seq\":%lu,\"code\":\"%s\",\"msg\":\"%s\"}",
                       i == 0 ? "" : ",",
                       (long long)entries[i].timestamp_us,
                       (unsigned long)entries[i].seq,
                       entries[i].code, entries[i].message);
        }
        comm_send("]\n");
        return;
    }
    if (strcmp(line, "LOG_CLEAR") == 0) {
        error_log_clear();
        comm_send("OK\n");
        return;
    }
    /* GET FAULT_TRACE <axis>: 10ms間隔・直近400件（4秒分）のstate/pos/enc/diff/vel/電流トレース */
    if (sscanf(line, "GET FAULT_TRACE %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            static fault_trace_sample_t trace[FAULT_TRACE_CAPACITY];
            uint16_t n = motor_get_fault_trace((uint8_t)axis, trace, FAULT_TRACE_CAPACITY);
            comm_sendf("OK {\"interval_ms\":%u,\"axis\":%u,\"count\":%u,\"samples\":[",
                       (unsigned)FAULT_TRACE_INTERVAL_MS, (unsigned)axis, (unsigned)n);
            /* CommTaskのスタック(4096B)を圧迫しないよう static でBSSに置く（dispatch()内の
               他コマンドのローカル変数と合わせてスタックオーバーフローを起こした実績あり） */
            static char chunk[224];
            size_t used = 0;
            for (uint16_t i = 0; i < n; i++) {
                /* worst case ("," + "[" + 1桁state + 4×int32(符号込み最大11桁) + 1桁×3カンマ
                   + current_mA(最大6桁) + "]") ≈ 59 バイト。余裕を見て80。 */
                static char item[80];
                int len = snprintf(item, sizeof(item), "%s[%d,%ld,%ld,%ld,%ld,%d]",
                                   i == 0 ? "" : ",",
                                   (int)trace[i].state, (long)trace[i].step_pos,
                                   (long)trace[i].enc_steps, (long)trace[i].diff,
                                   (long)trace[i].vel, (int)trace[i].current_mA);
                if (len < 0) len = 0;
                if (len >= (int)sizeof(item)) len = (int)sizeof(item) - 1;   /* snprintf切り詰め時の安全弁 */
                if (used + (size_t)len >= sizeof(chunk)) {
                    chunk[used] = '\0';
                    comm_send(chunk);
                    used = 0;
                }
                memcpy(chunk + used, item, (size_t)len);
                used += (size_t)len;
            }
            chunk[used] = '\0';
            comm_send(chunk);
            comm_send("]}\n");
        }
        return;
    }
    if (strcmp(line, "GET BOARD_ID") == 0) {
        comm_sendf("OK %s\n", telemetry_board_id());
        return;
    }
    if (strcmp(line, "GET HOLD_MODE") == 0) {
        comm_sendf("OK %u\n", (unsigned)motor_get_hold_mode());
        return;
    }
    /* GET STALL_FAULT <axis>: 脱調フォルト閾値取得（軸ごと、F-MOT-08） */
    if (sscanf(line, "GET STALL_FAULT %u", &axis) == 1) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK %lu\n", (unsigned long)motor_get_stall_fault_th((uint8_t)axis));
        }
        return;
    }
    /* GET CURRENT_LIMIT: 過電流フォルト閾値取得（全軸共通・単一ADCセンサのため） */
    if (strcmp(line, "GET CURRENT_LIMIT") == 0) {
        comm_sendf("OK %.1f\n", (double)adc_get_overcurrent_th());
        return;
    }
    /* GET MICROSTEP: マイクロステップ分周比取得（全軸共通、M0/M1/M2が全軸共通GPIOのため） */
    if (strcmp(line, "GET MICROSTEP") == 0) {
        comm_sendf("OK %d\n", (int)config_get_microstep());
        return;
    }
    if (strcmp(line, "GET HOLD_CURRENT_PERCENT") == 0) {
        comm_sendf("OK %u\n", (unsigned)motor_get_hold_current_percent());
        return;
    }
    if (strcmp(line, "GET BLE_STATUS") == 0) {
        switch (ble_telemetry_get_status()) {
        case BLE_TELEMETRY_CONNECTED:
            comm_send("OK CONNECTED\n");
            break;
        case BLE_TELEMETRY_ADVERTISING:
            comm_send("OK ADVERTISING\n");
            break;
        case BLE_TELEMETRY_DISABLED:
            comm_send("OK DISABLED\n");
            break;
        default:
            comm_send("ERR E016 BLE_INIT_FAILED\n");
            break;
        }
        return;
    }
    if (strcmp(line, "GET WIFI_STATUS") == 0) {
        char status[64];
        wifi_telemetry_format_status(status, sizeof(status));
        comm_sendf("OK %s\n", status);
        return;
    }

    /* STATUS — 全軸サマリー */
    if (strcmp(line, "STATUS") == 0) {
        comm_sendf("{\"microstep\":\"1/%d\",\"axes\":[",
                   (int)config_get_microstep());
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            axis_status_t st;
            motor_get_status(i, &st);
            comm_sendf("%s{\"id\":%u,\"state\":\"%s\",\"pos\":%ld,"
                       "\"vel\":%ld,\"vmax\":%lu,\"accel\":%lu,\"decel\":%lu}",
                       (i > 0 ? "," : ""),
                       (unsigned)i, state_name(st.state),
                       (long)st.pos, (long)st.vel,
                       (unsigned long)st.v_max,
                       (unsigned long)st.accel,
                       (unsigned long)st.decel);
        }
        comm_send("]}\n");
        return;
    }

    /* ---------- SET ---------- */
    unsigned val;
    /* モーション中の SET は拒否 */
    if (strncmp(line, "SET ", 4) == 0) {
        args = command_args(line, "SET POT_SCALE");
        if (args) {
            long parsed_axis;
            float scale;
            if (!parse_axis_float_args(args, &parsed_axis, &scale)) {
                comm_send("ERR E002 INVALID_ARG\n");
            } else if (parsed_axis < 0 || parsed_axis >= NUM_AXES) {
                comm_send("ERR E003 INVALID_AXIS\n");
            } else if (!isfinite(scale) || scale <= 0.0f) {
                comm_send("ERR E002 INVALID_PARAM\n");
            } else {
                comm_send(config_set_pot_scale_deg((uint8_t)parsed_axis, scale)
                          ? "OK\n" : "ERR E007 NVS_FAIL\n");
            }
            return;
        }

        args = command_args(line, "SET POT_ZERO");
        if (args) {
            long parsed_axis;
            if (!parse_axis_arg(args, &parsed_axis)) {
                comm_send("ERR E002 INVALID_ARG\n");
            } else if (parsed_axis < 0 || parsed_axis >= NUM_AXES) {
                comm_send("ERR E003 INVALID_AXIS\n");
            } else {
                int raw = adc_get_raw_count((uint8_t)parsed_axis);
                comm_send(config_set_pot_zero_offset((uint8_t)parsed_axis, raw)
                          ? "OK\n" : "ERR E007 NVS_FAIL\n");
            }
            return;
        }

        /* COMM_TIMEOUT と ADC_FILTER はモーション中でも変更可 — 先に処理 */
        if (sscanf(line, "SET COMM_TIMEOUT %u", &val) == 1) {
            s_wdog_ms      = val;
            s_wdog_enabled = (val > 0);
            s_last_rx_us   = esp_timer_get_time();   /* タイマーリセット */
            config_set_comm_timeout(val);
            comm_send("OK\n");
            return;
        }
        if (sscanf(line, "SET IDLE_TIMEOUT %u", &val) == 1) {
            motor_set_idle_timeout(val);
            config_set_idle_timeout(val);
            comm_send("OK\n");
            return;
        }
        if (sscanf(line, "SET HOLD_MODE %u", &val) == 1) {
            if (!motor_set_hold_mode((hold_mode_t)val)) {
                comm_send("ERR E002 INVALID_ARG\n");
            } else {
                config_set_hold_mode(val);
                comm_send("OK\n");
            }
            return;
        }
        if (sscanf(line, "SET HOLD_CURRENT_PERCENT %u", &val) == 1) {
            if (val < 1 || val > 100 || !motor_set_hold_current_percent((uint8_t)val)) {
                comm_send("ERR E002 INVALID_ARG\n");
            } else {
                config_set_hold_current_percent(val);
                comm_send("OK\n");
            }
            return;
        }

        if (sscanf(line, "SET BLE_ENABLE %u", &val) == 1) {
            if (val > 1) {
                comm_send("ERR E002 INVALID_ARG\n");
            } else if (!config_set_ble_enable(val != 0)) {
                comm_send("ERR E007 NVS_FAIL\n");
            } else if (ble_telemetry_set_enabled(val != 0) != ESP_OK) {
                comm_send("ERR E016 BLE_INIT_FAILED\n");
            } else {
                comm_send("OK\n");
            }
            return;
        }
        if (strncmp(line, "SET WIFI_SSID ", 14) == 0) {
            const char *ssid = line + 14;
            if (!config_set_wifi_ssid(ssid)) {
                comm_send(strlen(ssid) == 0 || strlen(ssid) > 32
                              ? "ERR E002 INVALID_ARG\n"
                              : "ERR E007 NVS_FAIL\n");
            } else if (wifi_telemetry_reconfigure() != ESP_OK) {
                comm_send("ERR E007 WIFI_RECONFIGURE_FAILED\n");
            } else {
                comm_send("OK\n");
            }
            return;
        }
        if (strncmp(line, "SET WIFI_PASS ", 14) == 0) {
            const char *password = line + 14;
            if (strlen(password) > 64) {
                comm_send("ERR E002 INVALID_ARG\n");
            } else if (!config_set_wifi_password(password)) {
                comm_send("ERR E007 NVS_FAIL\n");
            } else if (wifi_telemetry_reconfigure() != ESP_OK) {
                comm_send("ERR E007 WIFI_RECONFIGURE_FAILED\n");
            } else {
                comm_send("OK\n");
            }
            return;
        }
        if (sscanf(line, "SET WIFI_ENABLE %u", &val) == 1) {
            if (val > 1) {
                comm_send("ERR E002 INVALID_ARG\n");
            } else if (val == 1 && config_get_wifi_ssid()[0] == '\0') {
                comm_send("ERR E015 WIFI_NOT_CONFIGURED\n");
            } else if (!config_set_wifi_enable(val != 0)) {
                comm_send("ERR E007 NVS_FAIL\n");
            } else if (wifi_telemetry_set_enabled(val != 0) != ESP_OK) {
                comm_send("ERR E007 WIFI_STATE_CHANGE_FAILED\n");
            } else {
                comm_send("OK\n");
            }
            return;
        }
        if (sscanf(line, "SET WIFI_TELEMETRY_RATE %u", &val) == 1) {
            if (val < 1 || val > 100) {
                comm_send("ERR E002 INVALID_ARG\n");
            } else if (!config_set_wifi_telemetry_rate((uint8_t)val)) {
                comm_send("ERR E007 NVS_FAIL\n");
            } else {
                comm_send("OK\n");
            }
            return;
        }

        /* ADC 関連はモーション中でも変更可（監視・安全機能） */
        {
            char ch_str[8];
            unsigned filter_n;
            if (sscanf(line, "SET ADC_FILTER %7s %u", ch_str, &filter_n) == 2) {
                uint8_t ch = (strcmp(ch_str, "ALL") == 0) ? 0xFF : (uint8_t)atoi(ch_str);
                if (ch != 0xFF && ch > 4) {
                    comm_send("ERR E002 INVALID_ARG\n");
                } else if (filter_n < 1 || filter_n > 64) {
                    comm_send("ERR E002 INVALID_ARG\n");
                } else {
                    adc_set_filter_window(ch, (uint8_t)filter_n);
                    comm_send("OK\n");
                }
                return;
            }
        }
        {
            float mA_val;
            if (sscanf(line, "SET CURRENT_LIMIT %u %f", &axis, &mA_val) == 2) {
                if (mA_val <= 0.0f) {
                    comm_send("ERR E002 INVALID_ARG\n");
                } else {
                    adc_set_overcurrent_th(mA_val);
                    comm_send("OK\n");
                }
                return;
            }
        }
        {
            float ratio;
            if (sscanf(line, "SET VOLT_DIVIDER %f", &ratio) == 1) {
                if (ratio <= 0.0f) {
                    comm_send("ERR E002 INVALID_ARG\n");
                } else {
                    adc_set_volt_divider(ratio);
                    comm_send("OK\n");
                }
                return;
            }
        }

        /* 上記以外の SET はモーション中に拒否 */
        bool any_moving = false;
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            if (motor_is_moving(i)) { any_moving = true; break; }
        }
        if (any_moving) {
            comm_send("ERR E004 MOTION_IN_PROGRESS\n");
            return;
        }

        if (sscanf(line, "SET VMAX %u %u", &axis, &val) == 2) {
            comm_send(motor_set_vmax((uint8_t)axis, val) ? "OK\n" : "ERR E002 INVALID_PARAM\n");
            return;
        }
        if (sscanf(line, "SET ACCEL %u %u", &axis, &val) == 2) {
            comm_send(motor_set_accel((uint8_t)axis, val) ? "OK\n" : "ERR E002 INVALID_PARAM\n");
            return;
        }
        if (sscanf(line, "SET DECEL %u %u", &axis, &val) == 2) {
            comm_send(motor_set_decel((uint8_t)axis, val) ? "OK\n" : "ERR E002 INVALID_PARAM\n");
            return;
        }
        if (sscanf(line, "SET MICROSTEP %u", &val) == 1) {
            comm_send(config_set_microstep((microstep_t)val) ? "OK\n" : "ERR E002 INVALID_PARAM\n");
            return;
        }
        {
            float gear_value;
            if (sscanf(line, "SET GEAR_OFFSET %u %f", &axis, &gear_value) == 2) {
                if (axis >= NUM_AXES) {
                    comm_send("ERR E003 INVALID_AXIS\n");
                } else if (!isfinite(gear_value)) {
                    comm_send("ERR E002 INVALID_PARAM\n");
                } else {
                    comm_send(config_set_gear_offset((uint8_t)axis, gear_value)
                              ? "OK\n" : "ERR E007 NVS_FAIL\n");
                }
                return;
            }
        }
        {
            int gear_dir;
            if (sscanf(line, "SET GEAR_DIR %u %d", &axis, &gear_dir) == 2) {
                if (axis >= NUM_AXES) {
                    comm_send("ERR E003 INVALID_AXIS\n");
                } else if (gear_dir != 1 && gear_dir != -1) {
                    comm_send("ERR E002 INVALID_PARAM\n");
                } else {
                    comm_send(config_set_gear_dir((uint8_t)axis, (int8_t)gear_dir)
                              ? "OK\n" : "ERR E007 NVS_FAIL\n");
                }
                return;
            }
        }
        {
            unsigned capable;
            if (sscanf(line, "SET GEAR_ABS_CAPABLE %u %u", &axis, &capable) == 2) {
                if (axis >= NUM_AXES) {
                    comm_send("ERR E003 INVALID_AXIS\n");
                } else if (capable > 1) {
                    comm_send("ERR E002 INVALID_PARAM\n");
                } else {
                    comm_send(config_set_gear_abs_capable((uint8_t)axis,
                                                          capable != 0)
                              ? "OK\n" : "ERR E007 NVS_FAIL\n");
                }
                return;
            }
        }
        {
            unsigned enable;
            if (sscanf(line, "SET GEAR_ENABLE %u %u", &axis, &enable) == 2) {
                if (axis >= NUM_AXES) {
                    comm_send("ERR E003 INVALID_AXIS\n");
                } else if (enable > 1) {
                    comm_send("ERR E002 INVALID_PARAM\n");
                } else {
                    comm_send(config_set_gear_enable((uint8_t)axis, enable != 0)
                              ? "OK\n" : "ERR E007 NVS_FAIL\n");
                }
                return;
            }
        }
        {
            float warn_deg;
            if (sscanf(line, "SET GEAR_DEVIATION_WARN %f", &warn_deg) == 1) {
                if (!isfinite(warn_deg) || warn_deg < 0.0f) {
                    comm_send("ERR E002 INVALID_PARAM\n");
                } else {
                    comm_send(config_set_gear_deviation_warn(warn_deg)
                              ? "OK\n" : "ERR E007 NVS_FAIL\n");
                }
                return;
            }
        }
        int soft_min, soft_max;
        if (sscanf(line, "SET SOFT_LIMIT %u %d %d", &axis, &soft_min, &soft_max) == 3) {
            comm_send(motor_set_soft_limit((uint8_t)axis, soft_min, soft_max)
                      ? "OK\n" : "ERR E002 INVALID_PARAM\n");
            return;
        }
        if (sscanf(line, "SET STALL_FAULT %u %u", &axis, &val) == 2) {
            comm_send(motor_set_stall_fault_th((uint8_t)axis, val)
                      ? "OK\n" : "ERR E002 INVALID_PARAM\n");
            return;
        }
        {
            float gear_ratio_val;
            if (sscanf(line, "SET GEAR_RATIO %u %f", &axis, &gear_ratio_val) == 2) {
                comm_send(config_set_gear_ratio((uint8_t)axis, gear_ratio_val)
                          ? "OK\n" : "ERR E002 INVALID_PARAM\n");
                return;
            }
        }
        {
            int enc_dir_val;
            if (sscanf(line, "SET ENC_DIR %u %d", &axis, &enc_dir_val) == 2) {
                comm_send(motor_set_enc_dir((uint8_t)axis, (int8_t)enc_dir_val)
                          ? "OK\n" : "ERR E002 INVALID_PARAM\n");
                return;
            }
        }
        {
            unsigned motor_type_val;
            if (sscanf(line, "SET MOTOR_TYPE %u %u", &axis, &motor_type_val) == 2) {
                if (axis >= NUM_AXES) {
                    comm_send("ERR E003 INVALID_AXIS\n");
                } else if (motor_type_val > (unsigned)MOTOR_TYPE_OPEN_LOOP) {
                    comm_send("ERR E002 INVALID_PARAM\n");
                } else if (motor_is_moving((uint8_t)axis)) {
                    comm_send("ERR E008 MOTION_IN_PROGRESS\n");
                } else {
                    comm_send(motor_set_motor_type((uint8_t)axis, (motor_type_t)motor_type_val)
                              ? "OK\n" : "ERR E002 INVALID_PARAM\n");
                }
                return;
            }
        }
        {
            unsigned driver_type_val;
            if (sscanf(line, "SET DRIVER_TYPE %u %u", &axis, &driver_type_val) == 2) {
                if (axis >= NUM_AXES) {
                    comm_send("ERR E003 INVALID_AXIS\n");
                } else if (driver_type_val > (unsigned)DRIVER_TYPE_EXTERNAL) {
                    comm_send("ERR E002 INVALID_PARAM\n");
                } else if (motor_is_moving((uint8_t)axis)) {
                    comm_send("ERR E008 MOTION_IN_PROGRESS\n");
                } else {
                    comm_send(motor_set_driver_type((uint8_t)axis, (driver_type_t)driver_type_val)
                              ? "OK\n" : "ERR E002 INVALID_PARAM\n");
                }
                return;
            }
        }
        int home_dir_val;
        if (sscanf(line, "SET HOME_DIR %u %d", &axis, &home_dir_val) == 2) {
            comm_send(motor_set_home_dir((uint8_t)axis, (int8_t)home_dir_val)
                      ? "OK\n" : "ERR E002 INVALID_PARAM\n");
            return;
        }

        comm_send("ERR E001 UNKNOWN_CMD\n");
        return;
    }

    args = command_args(line, "CLEAR POT_ZERO");
    if (args) {
        long parsed_axis;
        if (!parse_axis_arg(args, &parsed_axis)) {
            comm_send("ERR E002 INVALID_ARG\n");
        } else if (parsed_axis < 0 || parsed_axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_send(config_set_pot_zero_offset((uint8_t)parsed_axis, 0)
                      ? "OK\n" : "ERR E007 NVS_FAIL\n");
        }
        return;
    }

    /* ---------- NVS 永続化 ---------- */
    if (strcmp(line, "SAVE") == 0) {
        comm_send(config_save() ? "OK\n" : "ERR E007 NVS_FAIL\n");
        return;
    }
    if (strcmp(line, "LOAD") == 0) {
        config_init();
        (void)ble_telemetry_set_enabled(config_get_ble_enable());
        (void)wifi_telemetry_set_enabled(config_get_wifi_enable());
        comm_send("OK\n");
        return;
    }
    if (strcmp(line, "RESET_CONFIG") == 0) {
        config_reset();
        (void)ble_telemetry_set_enabled(true);
        (void)wifi_telemetry_set_enabled(false);
        comm_send("OK\n");
        return;
    }

    /* ---------- Phase 1 テスト API ---------- */
    unsigned freq;
    if (sscanf(line, "TEST_PULSE %u %u", &axis, &freq) == 2) {
        comm_send(motor_test_pulse((uint8_t)axis, freq) ? "OK\n" : "ERR E003 INVALID_AXIS\n");
        return;
    }
    if (sscanf(line, "TEST_STOP %u", &axis) == 1) {
        comm_send(motor_stop_immediate((uint8_t)axis) ? "OK\n" : "ERR E003 INVALID_AXIS\n");
        return;
    }
    unsigned count;
    if (sscanf(line, "TEST_GPIO %u %u", &axis, &count) == 2) {
        comm_send(motor_test_gpio_toggle((uint8_t)axis, count) ? "OK\n" : "ERR E003 INVALID_AXIS\n");
        return;
    }

    comm_send("ERR E001 UNKNOWN_CMD\n");
}

/* ------------------------------------------------------------------ */
/*  CommTask — VFS stdin 受信ループ                                     */
/* ------------------------------------------------------------------ */
static void comm_task(void *arg)
{
    char    line[COMM_CMD_MAX_LEN];
    uint8_t pos = 0;
    char    ch;

    for (;;) {
        int n = read(STDIN_FILENO, &ch, 1);
        if (n != 1) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* 受信タイムスタンプ更新 */
        s_last_rx_us = esp_timer_get_time();

        if (ch == '\r') continue;

        if (ch == '\n') {
            line[pos] = '\0';
            if (pos > 0) {
                status_led_note_usb_activity();
                dispatch(line);
            }
            pos = 0;
        } else {
            if (pos < COMM_CMD_MAX_LEN - 1) {
                line[pos++] = ch;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/*  公開 API                                                            */
/* ------------------------------------------------------------------ */
void comm_init(void)
{
    s_tx_mutex = xSemaphoreCreateMutex();

    /* motor_ctrl のイベントコールバックを登録 */
    motor_register_move_done_cb(on_move_done);
    motor_register_move_aborted_cb(on_move_aborted);
    motor_register_limit_hit_cb(on_limit_hit);
    motor_register_fault_cb(on_fault);
    motor_register_home_done_cb(on_home_done);
    motor_register_home_timeout_cb(on_home_timeout);

    /* ADC 過電流コールバックを登録 */
    adc_register_overcurrent_cb(on_overcurrent);

    /* SYNC_MOVE コールバックを登録 */
    motor_register_sync_done_cb(on_sync_done);
    motor_register_sync_aborted_cb(on_sync_aborted);
    gear_monitor_register_event_callback(on_gear_event);

    /* ウォッチドッグ初期値を config から取得 */
    s_wdog_ms      = config_get_comm_timeout();
    s_wdog_enabled = (s_wdog_ms > 0);

    /* アイドルタイムアウトを config から取得して motor_ctrl へ適用 */
    motor_set_idle_timeout(config_get_idle_timeout());

    ESP_LOGI(TAG, "CommTask starting (USB Serial/JTAG via VFS)");
    /* dispatch() は多数のGETコマンドを1関数で処理し、各コマンドのローカル配列
       （GET LOGのentries[24]≈1.8KB等）がスタックフレームに積み上がるため、
       4096Bでは実際にスタックオーバーフローが発生した。余裕を見て8192Bに拡大。 */
    xTaskCreate(comm_task,     "CommTask",     8192, NULL, 10, NULL);
    xTaskCreate(watchdog_task, "WatchdogTask", 2048, NULL,  5, NULL);
    xTaskCreate(status_task,   "StatusTask",   2048, NULL,  5, NULL);
}

/* comm_send() が良性(成功)イベントとして記録対象から除外する EVT 名 */
static const char *const BENIGN_EVENTS[] = {
    "MOVE_DONE", "HOME_DONE", "SYNC_DONE", "GEAR_RECOVERED", "GEAR_AVAILABLE",
};

static bool is_benign_event(const char *name, size_t name_len)
{
    for (size_t i = 0; i < sizeof(BENIGN_EVENTS) / sizeof(BENIGN_EVENTS[0]); i++) {
        if (strlen(BENIGN_EVENTS[i]) == name_len &&
            strncmp(name, BENIGN_EVENTS[i], name_len) == 0) {
            return true;
        }
    }
    return false;
}

/*
 * comm_send() は全ての ERR/EVT 応答が通る唯一のフックポイント。
 * ここで FAULT/エラー系イベントのみを error_log に記録する
 * (ESP_LOGI 相当の一般情報ログ・OK/HB/STATUS 等は対象外)。
 */
static void maybe_record_error_log(const char *msg)
{
    bool is_err = strncmp(msg, "ERR ", 4) == 0;
    bool is_evt = !is_err && strncmp(msg, "EVT ", 4) == 0;
    if (!is_err && !is_evt) return;

    const char *rest = msg + 4;
    const char *token_end = rest;
    while (*token_end && *token_end != ' ' && *token_end != '\n' && *token_end != '\r') token_end++;
    size_t token_len = (size_t)(token_end - rest);
    if (token_len == 0) return;

    if (is_evt && is_benign_event(rest, token_len)) return;

    char code[24];   /* error_log.h の error_log_entry_t.code と同じサイズを維持すること */
    size_t code_len = token_len < sizeof(code) - 1 ? token_len : sizeof(code) - 1;
    memcpy(code, rest, code_len);
    code[code_len] = '\0';

    const char *message = token_end;
    while (*message == ' ') message++;
    size_t message_len = strlen(message);
    while (message_len > 0 && (message[message_len - 1] == '\n' || message[message_len - 1] == '\r')) {
        message_len--;
    }
    char clean_message[40];
    if (message_len >= sizeof(clean_message)) message_len = sizeof(clean_message) - 1;
    memcpy(clean_message, message, message_len);
    clean_message[message_len] = '\0';

    error_log_record(code, clean_message);
}

void comm_send(const char *msg)
{
    maybe_record_error_log(msg);
    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    fputs(msg, stdout);
    fflush(stdout);
    xSemaphoreGive(s_tx_mutex);
}

void comm_sendf(const char *fmt, ...)
{
    char    buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    comm_send(buf);
}
