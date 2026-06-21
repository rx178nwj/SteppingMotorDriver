#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "gpio_config.h"
#include "motor_specs.h"

/* ------------------------------------------------------------------ */
/*  マイクロステップ分割数 → M2/M1/M0 ビットパターン (DRV8825)           */
/*  M2 M1 M0 : 分割数                                                  */
/*   0  0  0 : 1 (フルステップ)                                         */
/*   0  0  1 : 1/2                                                     */
/*   0  1  0 : 1/4                                                     */
/*   0  1  1 : 1/8                                                     */
/*   1  0  0 : 1/16                                                    */
/*   1  0  1 : 1/32 (デフォルト)                                        */
/* ------------------------------------------------------------------ */
typedef enum {
    MICROSTEP_1   = 1,
    MICROSTEP_2   = 2,
    MICROSTEP_4   = 4,
    MICROSTEP_8   = 8,
    MICROSTEP_16  = 16,
    MICROSTEP_32  = 32,
} microstep_t;

/* ------------------------------------------------------------------ */
/*  公開 API                                                            */
/* ------------------------------------------------------------------ */
void config_init(void);   /* NVS から読み込み、motor_ctrl へ適用 */
bool config_save(void);   /* 現在値を NVS へ書き込み */
void config_reset(void);  /* デフォルト値に戻して NVS をクリア */

/* マイクロステップ */
bool        config_set_microstep(microstep_t div);
microstep_t config_get_microstep(void);

/* アイドルタイムアウト (ms) */
void     config_set_idle_timeout(uint32_t ms);
uint32_t config_get_idle_timeout(void);

/* 通信ウォッチドッグタイムアウト (ms, 0=無効) */
void     config_set_comm_timeout(uint32_t ms);
uint32_t config_get_comm_timeout(void);

/* ------------------------------------------------------------------ */
/*  モータープロファイル (軸ごと)                                         */
/* ------------------------------------------------------------------ */

/* プロファイル切り替え。
   vmax/accel/decel をプロファイルのデフォルト値で上書きして NVS へ保存。
   axis: 0〜NUM_AXES-1 / 戻り値: false=無効な axis または id            */
bool config_set_motor_profile(uint8_t axis, motor_profile_id_t id);

/* 現在設定されているプロファイル ID を返す */
motor_profile_id_t config_get_motor_profile(uint8_t axis);

/* 現在のプロファイル構造体ポインタを返す (NONE の場合は NULL) */
const motor_profile_t *config_get_motor_profile_data(uint8_t axis);
