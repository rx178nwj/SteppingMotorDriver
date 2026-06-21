#pragma once
#include <stdint.h>

/* ====================================================================
   ADC モニター (Phase 4)
   - ADC1 5ch サンプリング（POT0〜2 / MOT_V / CURRENT）
   - 移動平均フィルタ（リングバッファ、N=1〜64）
   - 電流・電圧換算
   - 過電流保護（閾値超過で motor_estop(FAULT_OVERCURRENT)）
   ==================================================================== */

/* 過電流コールバック: 閾値超過時に電流値 [mA] を渡して呼ばれる */
typedef void (*adc_overcurrent_cb_t)(float current_mA);

/* 初期化 — ADCTask 起動（優先度 15、10 ms 周期） */
void  adc_monitor_init(void);

/* 過電流コールバック登録 */
void  adc_register_overcurrent_cb(adc_overcurrent_cb_t cb);

/* 換算済み電流 [mA]（CH4 = GPIO5、フィルタ後） */
float adc_get_current_mA(void);

/* 換算済み電源電圧 [V]（CH3 = GPIO4、フィルタ後） */
float adc_get_voltage_V(void);

/* フィルタ後生 mV 値（ch=0〜4） */
int   adc_get_raw_mv(uint8_t ch);

/* 移動平均窓サイズ設定 (ch=0〜4、0xFF=全チャンネル一括、N=1〜64) */
void  adc_set_filter_window(uint8_t ch, uint8_t n);

/* 過電流閾値 [mA] 設定 / 取得 */
void  adc_set_overcurrent_th(float mA);
float adc_get_overcurrent_th(void);

/* 電源電圧分圧比設定 / 取得 (V_mot = V_adc * ratio) */
void  adc_set_volt_divider(float ratio);
float adc_get_volt_divider(void);
