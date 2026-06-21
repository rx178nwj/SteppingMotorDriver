#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ====================================================================
   エンコーダードライバー (3ch, PCNT 4逓倍 A/B + Z相 GPIO ISR)

   位置単位 : counts  (= PPR × 4、例: ME1K 1000PPR → 4000 counts/rev)
   速度単位 : counts/sec
   ==================================================================== */

/* 初期化 — PCNT ユニット生成・Z相 GPIO 割り込み登録 */
void    encoder_init(void);

/* 位置取得 [counts]、32bit 符号付き */
int32_t encoder_get_pos(uint8_t axis);

/* 位置セット（ホーミング完了時・原点リセット） */
void    encoder_set_pos(uint8_t axis, int32_t pos);

/* 速度取得 [counts/sec]、encoder_update_10ms() 呼び出し後に有効 */
int32_t encoder_get_vel_cps(uint8_t axis);

/* Z相イベント確認（読み出しでフラグクリア）
   ホーミング動作中に MotorControlTask からポーリングする */
bool    encoder_take_z_event(uint8_t axis);

/* MotorControlTask から 10ms 毎に呼び出す
   差分カウント法で速度を更新し、200ms 無変化時に 0 に落とす */
void    encoder_update_10ms(void);
