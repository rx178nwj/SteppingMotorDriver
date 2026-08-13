#include "error_log.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"

static portMUX_TYPE        s_mux = portMUX_INITIALIZER_UNLOCKED;
static error_log_entry_t   s_entries[ERROR_LOG_CAPACITY];
static uint32_t            s_count = 0;  /* 有効エントリ数 (<= CAPACITY) */
static uint32_t            s_head  = 0;  /* 次に書き込むインデックス */
static uint32_t            s_seq   = 0;  /* 直近発行した seq（clear() でもリセットしない） */

void error_log_init(void)
{
    taskENTER_CRITICAL(&s_mux);
    memset(s_entries, 0, sizeof(s_entries));
    s_count = 0;
    s_head  = 0;
    s_seq   = 0;
    taskEXIT_CRITICAL(&s_mux);
}

void error_log_record(const char *code, const char *message)
{
    if (!code) return;
    taskENTER_CRITICAL(&s_mux);
    error_log_entry_t *entry = &s_entries[s_head];
    entry->timestamp_us = esp_timer_get_time();
    entry->seq = ++s_seq;
    strncpy(entry->code, code, sizeof(entry->code) - 1);
    entry->code[sizeof(entry->code) - 1] = '\0';
    strncpy(entry->message, message ? message : "", sizeof(entry->message) - 1);
    entry->message[sizeof(entry->message) - 1] = '\0';
    s_head = (s_head + 1) % ERROR_LOG_CAPACITY;
    if (s_count < ERROR_LOG_CAPACITY) s_count++;
    taskEXIT_CRITICAL(&s_mux);
}

uint32_t error_log_get_seq(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint32_t seq = s_seq;
    taskEXIT_CRITICAL(&s_mux);
    return seq;
}

bool error_log_get_latest(error_log_entry_t *out)
{
    if (!out) return false;
    bool has_entry = false;
    taskENTER_CRITICAL(&s_mux);
    if (s_count > 0) {
        uint32_t idx = (s_head + ERROR_LOG_CAPACITY - 1) % ERROR_LOG_CAPACITY;
        *out = s_entries[idx];
        has_entry = true;
    }
    taskEXIT_CRITICAL(&s_mux);
    return has_entry;
}

uint32_t error_log_dump(error_log_entry_t *out, uint32_t max_entries)
{
    if (!out || max_entries == 0) return 0;
    taskENTER_CRITICAL(&s_mux);
    uint32_t count  = s_count;
    uint32_t oldest = (s_head + ERROR_LOG_CAPACITY - count) % ERROR_LOG_CAPACITY;
    uint32_t copied = count < max_entries ? count : max_entries;
    /* max_entries が保持件数より小さい場合は新しい方を優先して返す */
    uint32_t skip  = count - copied;
    uint32_t start = (oldest + skip) % ERROR_LOG_CAPACITY;
    for (uint32_t i = 0; i < copied; i++) {
        out[i] = s_entries[(start + i) % ERROR_LOG_CAPACITY];
    }
    taskEXIT_CRITICAL(&s_mux);
    return copied;
}

void error_log_clear(void)
{
    taskENTER_CRITICAL(&s_mux);
    s_count = 0;
    s_head  = 0;
    taskEXIT_CRITICAL(&s_mux);
}
