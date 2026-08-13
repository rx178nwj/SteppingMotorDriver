#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "gpio_config.h"
#include "motor_ctrl.h"
#include "encoder.h"
#include "adc_monitor.h"
#include "config.h"
#include "comm.h"
#include "error_log.h"
#include "gear_monitor.h"
#include "ble_telemetry.h"
#include "wifi_telemetry.h"
#include "status_led.h"

static const char *TAG = "main";

/* ------------------------------------------------------------------ */
/*  安全初期状態 — app_main() の最初の処理                               */
/*  GPIO の不定状態から意図しない励磁・パルス出力を防ぐ                    */
/* ------------------------------------------------------------------ */
static void gpio_safe_init(void)
{
    /* 出力 GPIO を一括設定 */
    const gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << GPIO_DRV_EN)    |
                        (1ULL << GPIO_DRV_RESET)  |
                        (1ULL << GPIO_DRV_SLEEP)  |
                        (1ULL << GPIO_STEP0)      |
                        (1ULL << GPIO_STEP1)      |
                        (1ULL << GPIO_STEP2)      |
                        (1ULL << GPIO_DIR0)       |
                        (1ULL << GPIO_DIR1)       |
                        (1ULL << GPIO_DIR2)       |
                        (1ULL << GPIO_M0)         |
                        (1ULL << GPIO_M1)         |
                        (1ULL << GPIO_M2)         |
                        (1ULL << GPIO_STATUS1_LED)|
                        (1ULL << GPIO_STATUS2_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);

    /* 安全初期状態を確定 */
    gpio_set_level(GPIO_DRV_EN,    1);   /* 非励磁 (アクティブ Low) */
    gpio_set_level(GPIO_DRV_SLEEP, 0);   /* スリープ (アクティブ High) */
    gpio_set_level(GPIO_DRV_RESET, 0);   /* リセット保持 (アクティブ Low) */
    gpio_set_level(GPIO_STEP0, 0);
    gpio_set_level(GPIO_STEP1, 0);
    gpio_set_level(GPIO_STEP2, 0);
    gpio_set_level(GPIO_DIR0, 0);
    gpio_set_level(GPIO_DIR1, 0);
    gpio_set_level(GPIO_DIR2, 0);
    /* マイクロステップ: デフォルト 1/32 (M2=H, M1=L, M0=H) */
    gpio_set_level(GPIO_M0, 1);
    gpio_set_level(GPIO_M1, 0);
    gpio_set_level(GPIO_M2, 1);
    gpio_set_level(GPIO_STATUS1_LED, 0);
    gpio_set_level(GPIO_STATUS2_LED, 0);
}

/* ------------------------------------------------------------------ */
/*  DRV8825 完全起動シーケンス (F-MOT-03)                               */
/* ------------------------------------------------------------------ */
static void drv8825_startup_sequence(void)
{
    /* Step 1: 安全状態確保（gpio_safe_init() 済み） */
    vTaskDelay(pdMS_TO_TICKS(1));               /* Step 2 */

    gpio_set_level(GPIO_DRV_RESET, 1);          /* Step 3: RESET 解除 */
    vTaskDelay(pdMS_TO_TICKS(1));               /* Step 4: 内部初期化完了待ち */

    gpio_set_level(GPIO_DRV_SLEEP, 1);          /* Step 5: SLEEP 解除 */
    vTaskDelay(pdMS_TO_TICKS(1));               /* Step 6: チャージポンプ安定待ち */

    /* Step 7: NVS から設定読み込み（Phase 2 で実装、現在はデフォルト値を使用） */
    ESP_LOGI(TAG, "DRV8825 startup sequence complete");

    /* Step 8: M0/M1/M2 は gpio_safe_init() でデフォルト値設定済み */
    /* Step 9: DRV_EN は High のまま維持（励磁は ENABLE コマンドで行う） */
}

/* ------------------------------------------------------------------ */
/*  app_main                                                            */
/* ------------------------------------------------------------------ */
void app_main(void)
{
    /* --- 安全初期状態（最初の処理）--- */
    gpio_safe_init();

    /* --- NVS 初期化 --- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* --- DRV8825 起動シーケンス --- */
    drv8825_startup_sequence();

    /* --- RMT モーター制御初期化 --- */
    motor_ctrl_init();

    /* --- ADC モニター初期化 (5ch, ADCTask 起動) --- */
    adc_monitor_init();

    /* --- NVS からパラメータ読み込み・適用 (motor_type を含む。encoder_init() より先に
           実行し、オープンループ軸の PCNT/Z相 GPIO 初期化スキップ判定に使う) --- */
    config_init();

    /* --- PCNT エンコーダー初期化 (motor_type==CLOSED_LOOP の軸のみ) --- */
    encoder_init();

    /* --- エラーログリングバッファ初期化 (comm_send() が使用) --- */
    error_log_init();

    /* --- USB-CDC + CommTask 起動 --- */
    comm_init();

    /* --- ギア出力軸角度モニター (非同期、未接続でも起動継続) --- */
    gear_monitor_init();

    /* --- Read-only wireless telemetry (independent of motor control) --- */
    esp_err_t ble_err = ble_telemetry_init();
    if (ble_err != ESP_OK) {
        ESP_LOGE(TAG, "E016 BLE_INIT_FAILED: %s", esp_err_to_name(ble_err));
    }
    esp_err_t wifi_err = wifi_telemetry_init();
    if (wifi_err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi telemetry init failed: %s",
                 esp_err_to_name(wifi_err));
    }

    ESP_ERROR_CHECK(status_led_init());

    ESP_LOGI(TAG, "SteppingMotorDriver firmware started");

    /* app_main は返らずに idle loop --- */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
