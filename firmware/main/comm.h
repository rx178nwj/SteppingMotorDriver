#pragma once

#define COMM_CMD_MAX_LEN  128

/* 初期化 (CommTask 起動) */
void comm_init(void);

/* USB-CDC へ文字列送信 (スレッドセーフ) */
void comm_send(const char *msg);

/* printf 形式で送信 */
void comm_sendf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
