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

/* 保持電流モード（全軸共通。値はmotor_ctrl.hのhold_mode_tに対応） */
void     config_set_hold_mode(uint32_t mode);
uint32_t config_get_hold_mode(void);
/* HOLD_REDUCEDのチョッピング比率 (1-100%) */
void     config_set_hold_current_percent(uint32_t pct);
uint32_t config_get_hold_current_percent(void);

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

/* ------------------------------------------------------------------ */
/*  ギア出力角度モニタ設定 (gear_config 名前空間)                         */
/* ------------------------------------------------------------------ */
float  config_get_gear_offset(uint8_t axis);
int8_t config_get_gear_dir(uint8_t axis);
bool   config_get_gear_abs_capable(uint8_t axis);
bool   config_get_gear_enable(uint8_t axis);
float  config_get_gear_deviation_warn(void);
float  config_get_gear_ratio(uint8_t axis);
/* F-MOT-10 パラメータ（RAM 反映のみ、NVS へは SAVE コマンドで永続化）。 */
bool   config_set_gear_ratio(uint8_t axis, float ratio);

/* F-MOT-12 関節角度換算・POT 較正。 */
uint32_t config_get_steps_per_rev(uint8_t axis);
uint32_t config_get_encoder_ppr(uint8_t axis);
float    config_get_pot_scale_deg(uint8_t axis);
int32_t  config_get_pot_zero_offset(uint8_t axis);

/* POT 較正値は SET コマンドから即時 NVS 保存する。 */
bool config_set_pot_scale_deg(uint8_t axis, float deg_per_count);
bool config_set_pot_zero_offset(uint8_t axis, int32_t adc_count);

/* SET GEAR_* は即時 NVS 保存する。false は引数不正または NVS 失敗。 */
bool config_set_gear_offset(uint8_t axis, float deg);
bool config_set_gear_dir(uint8_t axis, int8_t dir);
bool config_set_gear_abs_capable(uint8_t axis, bool capable);
bool config_set_gear_enable(uint8_t axis, bool enable);
bool config_set_gear_deviation_warn(float deg);

/* BLE / WiFi telemetry configuration (wifi_config namespace). */
bool        config_get_ble_enable(void);
bool        config_set_ble_enable(bool enable);
const char *config_get_wifi_ssid(void);
const char *config_get_wifi_password(void);
bool        config_get_wifi_enable(void);
uint8_t     config_get_wifi_telemetry_rate(void);
bool        config_set_wifi_ssid(const char *ssid);
bool        config_set_wifi_password(const char *password);
bool        config_set_wifi_enable(bool enable);
bool        config_set_wifi_telemetry_rate(uint8_t hz);
