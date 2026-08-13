#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/*  エラーログ（FAULT/エラー系イベントのみ。ESP_LOGI相当の一般ログは対象外） */
/* ------------------------------------------------------------------ */
#define ERROR_LOG_CAPACITY 24

typedef struct {
    int64_t  timestamp_us;  /* esp_timer_get_time() 基準（ブート起点） */
    uint32_t seq;           /* 単調増加シーケンス番号（欠落検出用） */
    /* "E005" / "COMM_TIMEOUT" / "OVERCURRENT" 等。EVTはイベント名全体がcodeになるため、
       最長の "GEAR_DEVIATION_WARN"（20文字）がヌル終端込みで収まるサイズにすること。
       comm.c の maybe_record_error_log() 内 char code[] と同じサイズを維持する。 */
    char     code[24];
    char     message[40];   /* 残りの詳細テキスト（空の場合あり） */
} error_log_entry_t;

void error_log_init(void);

/* comm.c の comm_send() から呼ばれる。ISR コンテキストからは呼ばない。 */
void error_log_record(const char *code, const char *message);

uint32_t error_log_get_seq(void);
bool     error_log_get_latest(error_log_entry_t *out);

/* 保持中のエントリを古い→新しい順に out へコピーする。実コピー件数を返す。 */
uint32_t error_log_dump(error_log_entry_t *out, uint32_t max_entries);

void error_log_clear(void);
