#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "gpio_config.h"

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
/*  軸ごとの永続化パラメータ                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    uint32_t   v_max;       /* steps/sec */
    uint32_t   accel;       /* steps/sec² */
    uint32_t   decel;       /* steps/sec² */
    microstep_t microstep;  /* 全軸共通 (NVS には軸0の値のみ保存) */
} axis_config_t;

/* ------------------------------------------------------------------ */
/*  公開 API                                                            */
/* ------------------------------------------------------------------ */
void config_init(void);   /* NVS から読み込み、motor_ctrl へ適用 */
bool config_save(void);   /* 現在値を NVS へ書き込み */

bool config_set_microstep(microstep_t div);
microstep_t config_get_microstep(void);
