#include "config.h"
#include "motor_ctrl.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG    = "config";
static const char *NVS_NS = "motor_cfg";

/* デフォルト値 */
#define DEF_VMAX       1000U
#define DEF_ACCEL       500U
#define DEF_DECEL       500U
#define DEF_MICROSTEP    32U

static microstep_t s_microstep = MICROSTEP_32;

/* ------------------------------------------------------------------ */
/*  マイクロステップ → M2/M1/M0 GPIO 出力                               */
/* ------------------------------------------------------------------ */
static void apply_microstep_gpio(microstep_t div)
{
    uint8_t m2, m1, m0;
    switch (div) {
    case MICROSTEP_1:  m2=0; m1=0; m0=0; break;
    case MICROSTEP_2:  m2=0; m1=0; m0=1; break;
    case MICROSTEP_4:  m2=0; m1=1; m0=0; break;
    case MICROSTEP_8:  m2=0; m1=1; m0=1; break;
    case MICROSTEP_16: m2=1; m1=0; m0=0; break;
    default:           m2=1; m1=0; m0=1; break;  /* 1/32 */
    }
    gpio_set_level(GPIO_M2, m2);
    gpio_set_level(GPIO_M1, m1);
    gpio_set_level(GPIO_M0, m0);
    ESP_LOGI(TAG, "Microstep 1/%d (M2=%d M1=%d M0=%d)", (int)div, m2, m1, m0);
}

/* ------------------------------------------------------------------ */
/*  config_init — NVS から読み込んで適用                                */
/* ------------------------------------------------------------------ */
void config_init(void)
{
    nvs_handle_t h;
    esp_err_t    err = nvs_open(NVS_NS, NVS_READONLY, &h);

    uint32_t ms = DEF_MICROSTEP;
    if (err == ESP_OK) {
        nvs_get_u32(h, "microstep", &ms);
    }

    /* 軸ごとに保存値を読み込む (存在しない場合はデフォルト値を使用) */
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        char     key[12];
        uint32_t av = DEF_VMAX, aa = DEF_ACCEL, ad = DEF_DECEL;
        if (err == ESP_OK) {
            snprintf(key, sizeof(key), "vmax%d",  i); nvs_get_u32(h, key, &av);
            snprintf(key, sizeof(key), "accel%d", i); nvs_get_u32(h, key, &aa);
            snprintf(key, sizeof(key), "decel%d", i); nvs_get_u32(h, key, &ad);
        }
        motor_set_vmax(i, av);
        motor_set_accel(i, aa);
        motor_set_decel(i, ad);
        ESP_LOGI(TAG, "Axis %d: vmax=%lu accel=%lu decel=%lu", i,
                 (unsigned long)av, (unsigned long)aa, (unsigned long)ad);
    }

    if (err == ESP_OK) {
        nvs_close(h);
    } else {
        ESP_LOGI(TAG, "NVS not found, using defaults");
    }

    s_microstep = (microstep_t)ms;
    apply_microstep_gpio(s_microstep);
}

/* ------------------------------------------------------------------ */
/*  config_save — 現在の軸0パラメータを NVS へ書き込み                  */
/* ------------------------------------------------------------------ */
bool config_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        axis_status_t st;
        if (!motor_get_status(i, &st)) continue;
        char key[12];
        snprintf(key, sizeof(key), "vmax%d",  i); nvs_set_u32(h, key, st.v_max);
        snprintf(key, sizeof(key), "accel%d", i); nvs_set_u32(h, key, st.accel);
        snprintf(key, sizeof(key), "decel%d", i); nvs_set_u32(h, key, st.decel);
    }
    nvs_set_u32(h, "microstep", (uint32_t)s_microstep);
    esp_err_t err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Config saved to NVS");
        return true;
    }
    ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
    return false;
}

/* ------------------------------------------------------------------ */
/*  config_set_microstep                                               */
/* ------------------------------------------------------------------ */
bool config_set_microstep(microstep_t div)
{
    if (div != MICROSTEP_1  && div != MICROSTEP_2  &&
        div != MICROSTEP_4  && div != MICROSTEP_8  &&
        div != MICROSTEP_16 && div != MICROSTEP_32) {
        return false;
    }
    s_microstep = div;
    apply_microstep_gpio(div);
    return true;
}

microstep_t config_get_microstep(void)
{
    return s_microstep;
}
