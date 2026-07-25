#include "comm.h"
#include "motor_ctrl.h"
#include "encoder.h"
#include "adc_monitor.h"
#include "config.h"
#include "gear_monitor.h"

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
    int steps;
    if (sscanf(line, "MOVE %u %d", &axis, &steps) == 2) {
        if (axis >= NUM_AXES) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else if (motor_is_moving((uint8_t)axis)) {
            comm_send("ERR E008 MOTION_IN_PROGRESS\n");
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

    /* ---------- GET ---------- */
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
        } else {
            comm_sendf("OK %ld\n", (long)encoder_get_pos((uint8_t)axis));
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
            int enc_dir_val;
            if (sscanf(line, "SET ENC_DIR %u %d", &axis, &enc_dir_val) == 2) {
                comm_send(motor_set_enc_dir((uint8_t)axis, (int8_t)enc_dir_val)
                          ? "OK\n" : "ERR E002 INVALID_PARAM\n");
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

    /* ---------- NVS 永続化 ---------- */
    if (strcmp(line, "SAVE") == 0) {
        comm_send(config_save() ? "OK\n" : "ERR E007 NVS_FAIL\n");
        return;
    }
    if (strcmp(line, "LOAD") == 0) {
        config_init();
        comm_send("OK\n");
        return;
    }
    if (strcmp(line, "RESET_CONFIG") == 0) {
        config_reset();
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
            if (pos > 0) dispatch(line);
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
    xTaskCreate(comm_task,     "CommTask",     4096, NULL, 10, NULL);
    xTaskCreate(watchdog_task, "WatchdogTask", 2048, NULL,  5, NULL);
    xTaskCreate(status_task,   "StatusTask",   2048, NULL,  5, NULL);
}

void comm_send(const char *msg)
{
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
