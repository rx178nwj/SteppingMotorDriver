#include "comm.h"
#include "motor_ctrl.h"
#include "config.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
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
    case AXIS_FAULT:  return "FAULT";
    default:          return "UNKNOWN";
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
            /* タイムアウト後は再受信まで発火しないようにリセット */
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

    /* ---------- 有効化 / 無効化 ---------- */
    if (strcmp(line, "ENABLE") == 0) {
        motor_enable();
        comm_send("OK\n");
        return;
    }
    if (strcmp(line, "DISABLE") == 0) {
        motor_disable();
        comm_send("OK\n");
        return;
    }

    /* ---------- モーション ---------- */
    int steps;
    if (sscanf(line, "MOVE %u %d", &axis, &steps) == 2) {
        comm_send(motor_move((uint8_t)axis, steps) ? "OK\n" : "ERR E003 INVALID_AXIS\n");
        return;
    }

    int pos;
    if (sscanf(line, "MOVETO %u %d", &axis, &pos) == 2) {
        comm_send(motor_moveto((uint8_t)axis, pos) ? "OK\n" : "ERR E003 INVALID_AXIS\n");
        return;
    }

    int vel;
    if (sscanf(line, "VEL %u %d", &axis, &vel) == 2) {
        comm_send(motor_vel((uint8_t)axis, vel) ? "OK\n" : "ERR E003 INVALID_AXIS\n");
        return;
    }

    /* ---------- 停止 ---------- */
    if (sscanf(line, "STOP %u", &axis) == 1) {
        comm_send(motor_stop((uint8_t)axis) ? "OK\n" : "ERR E003 INVALID_AXIS\n");
        return;
    }
    if (strcmp(line, "STOP ALL") == 0) {
        for (uint8_t i = 0; i < NUM_AXES; i++) motor_stop(i);
        comm_send("OK\n");
        return;
    }
    if (strcmp(line, "ESTOP") == 0) {
        motor_estop();
        comm_send("OK\n");
        return;
    }

    /* ---------- フォルト ---------- */
    if (strcmp(line, "CLEAR_FAULT") == 0) {
        motor_clear_fault();
        comm_send("OK\n");
        return;
    }

    /* ---------- GET ---------- */
    if (sscanf(line, "GET STATE %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK STATE %s\n", state_name(st.state));
        }
        return;
    }
    if (sscanf(line, "GET POS %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK POS %ld\n", (long)st.pos);
        }
        return;
    }
    if (sscanf(line, "GET VEL %u", &axis) == 1) {
        axis_status_t st;
        if (!motor_get_status((uint8_t)axis, &st)) {
            comm_send("ERR E003 INVALID_AXIS\n");
        } else {
            comm_sendf("OK VEL %ld\n", (long)st.vel);
        }
        return;
    }

    /* STATUS — 全軸サマリー */
    if (strcmp(line, "STATUS") == 0) {
        comm_sendf("OK STATUS microstep=1/%d\n", (int)config_get_microstep());
        for (uint8_t i = 0; i < NUM_AXES; i++) {
            axis_status_t st;
            motor_get_status(i, &st);
            comm_sendf("  axis%d state=%s pos=%ld vel=%ld vmax=%lu accel=%lu decel=%lu\n",
                       i, state_name(st.state),
                       (long)st.pos, (long)st.vel,
                       (unsigned long)st.v_max,
                       (unsigned long)st.accel,
                       (unsigned long)st.decel);
        }
        return;
    }

    /* ---------- SET ---------- */
    unsigned val;
    if (sscanf(line, "SET VMAX %u %u", &axis, &val) == 2) {
        comm_send(motor_set_vmax((uint8_t)axis, val) ? "OK\n" : "ERR E006 INVALID_PARAM\n");
        return;
    }
    if (sscanf(line, "SET ACCEL %u %u", &axis, &val) == 2) {
        comm_send(motor_set_accel((uint8_t)axis, val) ? "OK\n" : "ERR E006 INVALID_PARAM\n");
        return;
    }
    if (sscanf(line, "SET DECEL %u %u", &axis, &val) == 2) {
        comm_send(motor_set_decel((uint8_t)axis, val) ? "OK\n" : "ERR E006 INVALID_PARAM\n");
        return;
    }
    if (sscanf(line, "SET MICROSTEP %u", &val) == 1) {
        comm_send(config_set_microstep((microstep_t)val) ? "OK\n" : "ERR E006 INVALID_PARAM\n");
        return;
    }
    if (sscanf(line, "SET COMM_TIMEOUT %u", &val) == 1) {
        s_wdog_ms      = val;
        s_wdog_enabled = (val > 0);
        comm_send("OK\n");
        return;
    }

    /* ---------- NVS 永続化 ---------- */
    if (strcmp(line, "SAVE") == 0) {
        comm_send(config_save() ? "OK\n" : "ERR E007 NVS_FAIL\n");
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
    ESP_LOGI(TAG, "CommTask starting (USB Serial/JTAG via VFS)");
    xTaskCreate(comm_task,     "CommTask",  4096, NULL, 10, NULL);
    xTaskCreate(watchdog_task, "WatchdogTask", 2048, NULL, 5, NULL);
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
