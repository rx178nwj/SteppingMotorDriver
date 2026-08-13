#include "adc_monitor.h"
#include "motor_ctrl.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "adc_mon";

/* ------------------------------------------------------------------ */
/*  定数                                                               */
/* ------------------------------------------------------------------ */
#define ADC_NUM_CH       5
#define ADC_OVERSAMPLE   4
#define ADC_FILTER_MAX   64
#define ADC_FILTER_DEF   8
#define ADC_OC_TH_DEF    6000.0f          /* mA — デフォルト過電流閾値（回路最大 ~7920 mA） */
#define ADC_OC_HYST      0.85f            /* 復帰ヒステリシス係数 */
#define VOLT_DIV_DEF     (24.0f / 3.3f)  /* MOT_V 分圧比 */

/* ------------------------------------------------------------------ */
/*  電流センス回路定数（LT6106 + シャント抵抗）                           */
/*  V_adc = I[A] × R_SHUNT × (R_OUT / R_IN)                           */
/*  I[mA] = V_adc[mV] × R_IN / (R_SHUNT × R_OUT)                     */
/*        = V_adc[mV] × 120 / (0.010 × 5000) = V_adc[mV] × 2.4       */
/* ------------------------------------------------------------------ */
#define CURRENT_R_SHUNT   0.010f   /* シャント抵抗 [Ω] (10 mΩ) */
#define CURRENT_R_IN      120.0f   /* LT6106 Rin [Ω] */
#define CURRENT_R_OUT     5000.0f  /* LT6106 Rout [Ω] */
#define CURRENT_CONV_FACTOR \
    (CURRENT_R_IN / (CURRENT_R_SHUNT * CURRENT_R_OUT))  /* 2.4 mA/mV */

/* ch インデックス */
#define CH_POT0     0
#define CH_POT1     1
#define CH_POT2     2
#define CH_MOT_V    3
#define CH_CURRENT  4

/* ADC1 チャンネルマッピング (GPIO1〜5 = ADC1_CH0〜CH4) */
static const adc_channel_t k_channels[ADC_NUM_CH] = {
    ADC_CHANNEL_0,  /* GPIO1 POT0    */
    ADC_CHANNEL_1,  /* GPIO2 POT1    */
    ADC_CHANNEL_2,  /* GPIO3 POT2    */
    ADC_CHANNEL_3,  /* GPIO4 MOT_V   */
    ADC_CHANNEL_4,  /* GPIO5 CURRENT */
};

/* ------------------------------------------------------------------ */
/*  移動平均リングバッファ                                               */
/* ------------------------------------------------------------------ */
typedef struct {
    int32_t buf[ADC_FILTER_MAX];
    uint8_t head;     /* 次書き込み位置 */
    uint8_t n;        /* 窓サイズ */
    int32_t sum;      /* buf[0..n-1] の合計 */
    int32_t last;     /* 直近の出力 [mV] */
} ring_t;

/* バッファを fill_val で初期化（窓サイズ変更時も使用） */
static void ring_reset(ring_t *r, uint8_t n, int32_t fill_val)
{
    r->n    = (n < 1) ? 1 : (n > ADC_FILTER_MAX) ? ADC_FILTER_MAX : n;
    r->head = 0;
    r->sum  = fill_val * (int32_t)r->n;
    r->last = fill_val;
    for (int i = 0; i < r->n; i++) r->buf[i] = fill_val;
}

/* サンプルを追加し、フィルタ後の出力 [mV] を返す */
static int32_t ring_push(ring_t *r, int32_t mv)
{
    r->sum -= r->buf[r->head];
    r->buf[r->head] = mv;
    r->sum  += mv;
    r->head  = (uint8_t)((r->head + 1) % r->n);
    r->last  = r->sum / (int32_t)r->n;
    return r->last;
}

/* ------------------------------------------------------------------ */
/*  モジュール状態                                                       */
/* ------------------------------------------------------------------ */
static ring_t                   s_ring[ADC_NUM_CH];
static ring_t                   s_raw_ring[ADC_NUM_CH];
static SemaphoreHandle_t        s_mutex;
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t        s_cali_handle;
static bool                     s_cali_valid = false;

static volatile float           s_oc_th_mA  = ADC_OC_TH_DEF;
static volatile float           s_volt_div  = VOLT_DIV_DEF;
static volatile bool            s_oc_latch  = false;  /* 過電流ラッチ */

static adc_overcurrent_cb_t     s_oc_cb = NULL;

/* ------------------------------------------------------------------ */
/*  ADC キャリブレーション初期化                                         */
/* ------------------------------------------------------------------ */
static bool calibration_init(void)
{
    esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    {
        adc_cali_curve_fitting_config_t cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cfg, &s_cali_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "calibration: curve fitting");
            return true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    {
        adc_cali_line_fitting_config_t cfg = {
            .unit_id  = ADC_UNIT_1,
            .atten    = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cfg, &s_cali_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "calibration: line fitting");
            return true;
        }
    }
#endif

    (void)ret;
    ESP_LOGW(TAG, "calibration not supported, using linear approximation");
    return false;
}

/* ------------------------------------------------------------------ */
/*  ADCTask                                                             */
/* ------------------------------------------------------------------ */
static void adc_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        /* 5ch × 4 オーバーサンプル → 平均 → キャリブレーション → フィルタ */
        for (int ch = 0; ch < ADC_NUM_CH; ch++) {
            int32_t raw_sum = 0;
            for (int i = 0; i < ADC_OVERSAMPLE; i++) {
                int raw = 0;
                adc_oneshot_read(s_adc_handle, k_channels[ch], &raw);
                raw_sum += raw;
            }
            int avg_raw = (int)(raw_sum / ADC_OVERSAMPLE);

            int mv = 0;
            if (s_cali_valid) {
                adc_cali_raw_to_voltage(s_cali_handle, avg_raw, &mv);
            } else {
                /* キャリブレーション非対応時: 線形近似 (Vref=3300 mV, 12bit) */
                mv = (int)(avg_raw * 3300L / 4095);
            }

            xSemaphoreTake(s_mutex, portMAX_DELAY);
            ring_push(&s_raw_ring[ch], (int32_t)avg_raw);
            ring_push(&s_ring[ch], (int32_t)mv);
            xSemaphoreGive(s_mutex);
        }

        /* 過電流チェック（F-ADC-03） */
        float curr_mA = adc_get_current_mA();
        float th      = s_oc_th_mA;

        if (!s_oc_latch && curr_mA > th) {
            s_oc_latch = true;
            ESP_LOGW(TAG, "OVERCURRENT: %.1f mA (th=%.1f mA)", curr_mA, th);
            if (s_oc_cb) s_oc_cb(curr_mA);
            motor_estop(FAULT_OVERCURRENT);
        } else if (s_oc_latch && curr_mA < th * ADC_OC_HYST) {
            /* 電流が閾値×ヒステリシス以下に下がったらラッチ解除 */
            s_oc_latch = false;
        }
    }
}

/* ------------------------------------------------------------------ */
/*  公開 API 実装                                                       */
/* ------------------------------------------------------------------ */
void adc_monitor_init(void)
{
    s_mutex = xSemaphoreCreateMutex();

    /* リングバッファ初期化 */
    for (int i = 0; i < ADC_NUM_CH; i++) {
        ring_reset(&s_ring[i], ADC_FILTER_DEF, 0);
        ring_reset(&s_raw_ring[i], ADC_FILTER_DEF, 0);
    }

    /* ADC1 ワンショットユニット初期化 */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    /* 全チャンネルの設定 (12dB: 0〜3.1V、12bit) */
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    for (int i = 0; i < ADC_NUM_CH; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, k_channels[i], &ch_cfg));
    }

    /* キャリブレーション */
    s_cali_valid = calibration_init();

    ESP_LOGI(TAG, "ADC init done, cali=%s", s_cali_valid ? "yes" : "no");

    xTaskCreate(adc_task, "ADCTask", 2048, NULL, 15, NULL);
}

void adc_register_overcurrent_cb(adc_overcurrent_cb_t cb)
{
    s_oc_cb = cb;
}

float adc_get_current_mA(void)
{
    /* I[mA] = V_adc[mV] × CURRENT_CONV_FACTOR
       = V_adc[mV] × Rin / (R_shunt × Rout) = V_adc[mV] × 2.4 */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int32_t mv = s_ring[CH_CURRENT].last;
    xSemaphoreGive(s_mutex);
    return (float)mv * CURRENT_CONV_FACTOR;
}

float adc_get_voltage_V(void)
{
    /* V_mot[V] = V_adc[mV] / 1000 * divider_ratio */
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int32_t mv = s_ring[CH_MOT_V].last;
    xSemaphoreGive(s_mutex);
    return (float)mv / 1000.0f * s_volt_div;
}

int adc_get_raw_mv(uint8_t ch)
{
    if (ch >= ADC_NUM_CH) return -1;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int32_t mv = s_ring[ch].last;
    xSemaphoreGive(s_mutex);
    return (int)mv;
}

int adc_get_raw_count(uint8_t ch)
{
    if (ch >= ADC_NUM_CH) return -1;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int32_t raw = s_raw_ring[ch].last;
    xSemaphoreGive(s_mutex);
    return (int)raw;
}

void adc_set_filter_window(uint8_t ch, uint8_t n)
{
    if (n < 1)  n = 1;
    if (n > ADC_FILTER_MAX) n = ADC_FILTER_MAX;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (ch == 0xFF) {
        /* 全チャンネル一括 */
        for (int i = 0; i < ADC_NUM_CH; i++) {
            ring_reset(&s_ring[i], n, s_ring[i].last);
            ring_reset(&s_raw_ring[i], n, s_raw_ring[i].last);
        }
    } else if (ch < ADC_NUM_CH) {
        ring_reset(&s_ring[ch], n, s_ring[ch].last);
        ring_reset(&s_raw_ring[ch], n, s_raw_ring[ch].last);
    }
    xSemaphoreGive(s_mutex);
}

void adc_set_overcurrent_th(float mA)
{
    s_oc_th_mA = mA;
    s_oc_latch = false;  /* 閾値変更時はラッチをリセット */
}

float adc_get_overcurrent_th(void)
{
    return s_oc_th_mA;
}

void adc_set_volt_divider(float ratio)
{
    if (ratio > 0.0f) s_volt_div = ratio;
}

float adc_get_volt_divider(void)
{
    return s_volt_div;
}
