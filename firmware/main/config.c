#include "config.h"
#include "motor_ctrl.h"
#include "motor_specs.h"
#include "gpio_config.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG    = "config";
static const char *NVS_NS = "motor_config";
static const char *GEAR_NVS_NS = "gear_config";

/* ------------------------------------------------------------------ */
/*  共通デフォルト値                                                     */
/* ------------------------------------------------------------------ */
#define DEF_MICROSTEP          32U
#define DEF_IDLE_TIMEOUT_MS  2000U
#define DEF_COMM_TIMEOUT_MS  5000U
#define DEF_GEAR_DEVIATION_WARN_DEG 5.0f

/* 軸ごとのデフォルトプロファイル ID */
static const motor_profile_id_t DEF_PROFILE_PER_AXIS[NUM_AXES] = {
    MOTOR_CH0_DEFAULT_PROFILE,   /* CH0: 17HS08-1004-ME1K */
    MOTOR_CH1_DEFAULT_PROFILE,   /* CH1: 17HS15-1504-ME1K */
    MOTOR_CH2_DEFAULT_PROFILE,   /* CH2: 17HS15-1504-ME1K */
};

/* ------------------------------------------------------------------ */
/*  モジュール変数                                                       */
/* ------------------------------------------------------------------ */
static microstep_t         s_microstep           = MICROSTEP_32;
static uint32_t            s_idle_timeout_ms     = DEF_IDLE_TIMEOUT_MS;
static uint32_t            s_comm_timeout_ms     = DEF_COMM_TIMEOUT_MS;
static motor_profile_id_t  s_profile[NUM_AXES];
static float                s_gear_offset[NUM_AXES] = {0.0f, 0.0f, 0.0f};
static int8_t               s_gear_dir[NUM_AXES] = {1, 1, 1};
static bool                 s_gear_abs_capable[NUM_AXES] = {false, false, false};
static bool                 s_gear_enable[NUM_AXES] = {true, true, true};
static float                s_gear_ratio[NUM_AXES] = {1.0f, 1.0f, 1.0f};
static float                s_gear_deviation_warn = DEF_GEAR_DEVIATION_WARN_DEG;

static uint32_t float_to_u32(float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static float u32_to_float(uint32_t bits)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void load_gear_config(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(GEAR_NVS_NS, NVS_READONLY, &h);

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        s_gear_offset[i] = 0.0f;
        s_gear_dir[i] = 1;
        s_gear_abs_capable[i] = false;
        s_gear_enable[i] = true;

        if (err == ESP_OK) {
            char key[16];
            uint32_t bits;
            int8_t dir;
            uint8_t flag;

            snprintf(key, sizeof(key), "gear_off_%u", i);
            if (nvs_get_u32(h, key, &bits) == ESP_OK) {
                float value = u32_to_float(bits);
                if (isfinite(value)) s_gear_offset[i] = value;
            }
            snprintf(key, sizeof(key), "gear_dir_%u", i);
            if (nvs_get_i8(h, key, &dir) == ESP_OK && (dir == 1 || dir == -1)) {
                s_gear_dir[i] = dir;
            }
            snprintf(key, sizeof(key), "gear_abscap_%u", i);
            if (nvs_get_u8(h, key, &flag) == ESP_OK) {
                s_gear_abs_capable[i] = (flag != 0);
            }
            snprintf(key, sizeof(key), "gear_en_%u", i);
            if (nvs_get_u8(h, key, &flag) == ESP_OK) {
                s_gear_enable[i] = (flag != 0);
            }
        }
    }

    s_gear_deviation_warn = DEF_GEAR_DEVIATION_WARN_DEG;
    if (err == ESP_OK) {
        uint32_t bits;
        if (nvs_get_u32(h, "gear_devwarn", &bits) == ESP_OK) {
            float value = u32_to_float(bits);
            if (isfinite(value) && value >= 0.0f) {
                s_gear_deviation_warn = value;
            }
        }
        nvs_close(h);
    }
}

static bool save_gear_config(void)
{
    nvs_handle_t h;
    if (nvs_open(GEAR_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        char key[16];
        snprintf(key, sizeof(key), "gear_off_%u", i);
        nvs_set_u32(h, key, float_to_u32(s_gear_offset[i]));
        snprintf(key, sizeof(key), "gear_dir_%u", i);
        nvs_set_i8(h, key, s_gear_dir[i]);
        snprintf(key, sizeof(key), "gear_abscap_%u", i);
        nvs_set_u8(h, key, s_gear_abs_capable[i] ? 1U : 0U);
        snprintf(key, sizeof(key), "gear_en_%u", i);
        nvs_set_u8(h, key, s_gear_enable[i] ? 1U : 0U);
    }
    nvs_set_u32(h, "gear_devwarn", float_to_u32(s_gear_deviation_warn));

    esp_err_t err = nvs_commit(h);
    nvs_close(h);
    return err == ESP_OK;
}

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
/*  内部: プロファイルの vmax/accel/decel を motor_ctrl へ適用           */
/* ------------------------------------------------------------------ */
static void apply_profile_params(uint8_t axis, const motor_profile_t *p)
{
    motor_set_vmax(axis,  p->vmax_default);
    motor_set_accel(axis, p->accel_default);
    motor_set_decel(axis, p->decel_default);
    ESP_LOGI(TAG, "Axis %d [%s]: vmax=%lu accel=%lu decel=%lu",
             axis, p->model,
             (unsigned long)p->vmax_default,
             (unsigned long)p->accel_default,
             (unsigned long)p->decel_default);
}

/* ------------------------------------------------------------------ */
/*  config_init — NVS から読み込んで適用                                */
/* ------------------------------------------------------------------ */
void config_init(void)
{
    nvs_handle_t h;
    esp_err_t    err = nvs_open(NVS_NS, NVS_READONLY, &h);

    uint32_t ms      = DEF_MICROSTEP;
    uint32_t idle_ms = DEF_IDLE_TIMEOUT_MS;
    uint32_t comm_ms = DEF_COMM_TIMEOUT_MS;

    if (err == ESP_OK) {
        nvs_get_u32(h, "microstep",    &ms);
        nvs_get_u32(h, "idle_timeout", &idle_ms);
        nvs_get_u32(h, "comm_timeout", &comm_ms);
    }

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        char key[16];

        /* プロファイル ID を NVS から復元 */
        uint32_t prof_id = (uint32_t)DEF_PROFILE_PER_AXIS[i];
        if (err == ESP_OK) {
            snprintf(key, sizeof(key), "mprof%d", i);
            nvs_get_u32(h, key, &prof_id);
        }
        /* 範囲外は デフォルトに戻す */
        if (prof_id == MOTOR_PROFILE_NONE || prof_id >= MOTOR_PROFILE_COUNT) {
            prof_id = (uint32_t)DEF_PROFILE_PER_AXIS[i];
        }
        s_profile[i] = (motor_profile_id_t)prof_id;

        /* vmax / accel / decel: NVS 保存値があれば優先、なければプロファイル既定値 */
        const motor_profile_t *p = motor_spec_get_profile(s_profile[i]);
        uint32_t av = p ? p->vmax_default  : 10000U;
        uint32_t aa = p ? p->accel_default : 50000U;
        uint32_t ad = p ? p->decel_default : 50000U;

        if (err == ESP_OK) {
            snprintf(key, sizeof(key), "vmax%d",  i); nvs_get_u32(h, key, &av);
            snprintf(key, sizeof(key), "accel%d", i); nvs_get_u32(h, key, &aa);
            snprintf(key, sizeof(key), "decel%d", i); nvs_get_u32(h, key, &ad);
        }

        motor_set_vmax(i, av);
        motor_set_accel(i, aa);
        motor_set_decel(i, ad);

        /* --- F-MOT-10 追加パラメータ (Phase 5) --- */
        /* 脱調フォルト閾値 */
        uint32_t stall_th = 512U;
        if (err == ESP_OK) { snprintf(key, sizeof(key), "stall%d", i); nvs_get_u32(h, key, &stall_th); }
        motor_set_stall_fault_th(i, stall_th);

        /* ソフトリミット */
        int32_t min_p = -2000000, max_p = 2000000;
        if (err == ESP_OK) {
            uint32_t tmp;
            snprintf(key, sizeof(key), "minp%d", i);
            if (nvs_get_u32(h, key, &tmp) == ESP_OK) min_p = (int32_t)tmp;
            snprintf(key, sizeof(key), "maxp%d", i);
            if (nvs_get_u32(h, key, &tmp) == ESP_OK) max_p = (int32_t)tmp;
        }
        motor_set_soft_limit(i, min_p, max_p);

        /* ホーミングパラメータ */
        uint32_t v_coarse = 2000U, v_fine = 500U, back_off = 200U;
        int32_t  home_ofs = 0;
        int8_t   home_dir = -1;
        if (err == ESP_OK) {
            snprintf(key, sizeof(key), "hcoarse%d", i); nvs_get_u32(h, key, &v_coarse);
            snprintf(key, sizeof(key), "hfine%d",   i); nvs_get_u32(h, key, &v_fine);
            snprintf(key, sizeof(key), "hback%d",   i); nvs_get_u32(h, key, &back_off);
            uint32_t tmp;
            snprintf(key, sizeof(key), "hofs%d", i);
            if (nvs_get_u32(h, key, &tmp) == ESP_OK) home_ofs = (int32_t)tmp;
            snprintf(key, sizeof(key), "hdir%d", i);
            if (nvs_get_u32(h, key, &tmp) == ESP_OK) home_dir = (int8_t)(int32_t)tmp;
        }
        motor_set_home_params(i, v_coarse, v_fine, (int32_t)back_off, home_ofs);
        motor_set_home_dir(i, home_dir);

        /* エンコーダ方向 (enc_dir: +1=正, -1=逆接続) */
        uint32_t enc_dir_u32 = 1U;
        if (err == ESP_OK) {
            snprintf(key, sizeof(key), "encdir%d", i);
            nvs_get_u32(h, key, &enc_dir_u32);
        }
        int8_t enc_dir = (enc_dir_u32 == (uint32_t)(int32_t)-1) ? -1 : 1;
        motor_set_enc_dir(i, enc_dir);

        /* gear_ratio は既存 F-MOT-10 パラメータ。未保存時は 1.0。 */
        s_gear_ratio[i] = 1.0f;
        if (err == ESP_OK) {
            uint32_t ratio_bits;
            snprintf(key, sizeof(key), "gearratio%d", i);
            if (nvs_get_u32(h, key, &ratio_bits) == ESP_OK) {
                float ratio = u32_to_float(ratio_bits);
                if (isfinite(ratio) && ratio > 0.0f) s_gear_ratio[i] = ratio;
            }
        }

        /* mpc_x100: プロファイルと現在マイクロステップから計算 */
        if (p && p->encoder_counts_per_rev > 0) {
            uint32_t mpc = ((uint32_t)p->full_steps_per_rev * ms * 100U)
                           / p->encoder_counts_per_rev;
            motor_set_mpc_x100(i, (uint16_t)(mpc > 0xFFFFU ? 0xFFFFU : mpc));
        }

        ESP_LOGI(TAG, "Axis %d profile=%lu [%s] vmax=%lu accel=%lu decel=%lu stall=%lu enc_dir=%d",
                 i, (unsigned long)s_profile[i],
                 p ? p->model : "NONE",
                 (unsigned long)av, (unsigned long)aa, (unsigned long)ad,
                 (unsigned long)stall_th, (int)enc_dir);
    }

    if (err == ESP_OK) {
        nvs_close(h);
    } else {
        ESP_LOGI(TAG, "NVS not found, using defaults");
    }

    s_microstep       = (microstep_t)ms;
    s_idle_timeout_ms = idle_ms;
    s_comm_timeout_ms = comm_ms;

    apply_microstep_gpio(s_microstep);
    motor_set_idle_timeout(s_idle_timeout_ms);
    load_gear_config();

    ESP_LOGI(TAG, "idle_timeout=%lums comm_timeout=%lums",
             (unsigned long)idle_ms, (unsigned long)comm_ms);
}

/* ------------------------------------------------------------------ */
/*  config_save — 現在の設定を NVS へ書き込み                           */
/* ------------------------------------------------------------------ */
bool config_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return false;

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        axis_status_t st;
        if (!motor_get_status(i, &st)) continue;
        char key[16];
        snprintf(key, sizeof(key), "vmax%d",  i); nvs_set_u32(h, key, st.v_max);
        snprintf(key, sizeof(key), "accel%d", i); nvs_set_u32(h, key, st.accel);
        snprintf(key, sizeof(key), "decel%d", i); nvs_set_u32(h, key, st.decel);
        snprintf(key, sizeof(key), "mprof%d", i); nvs_set_u32(h, key, (uint32_t)s_profile[i]);

        /* F-MOT-10 追加パラメータ */
        snprintf(key, sizeof(key), "stall%d", i);
        nvs_set_u32(h, key, motor_get_stall_fault_th(i));

        int32_t min_p, max_p;
        motor_get_soft_limit(i, &min_p, &max_p);
        snprintf(key, sizeof(key), "minp%d", i); nvs_set_u32(h, key, (uint32_t)min_p);
        snprintf(key, sizeof(key), "maxp%d", i); nvs_set_u32(h, key, (uint32_t)max_p);

        snprintf(key, sizeof(key), "hcoarse%d", i); nvs_set_u32(h, key, motor_get_v_home_coarse(i));
        snprintf(key, sizeof(key), "hfine%d",   i); nvs_set_u32(h, key, motor_get_v_home_fine(i));
        snprintf(key, sizeof(key), "hback%d",   i); nvs_set_u32(h, key, (uint32_t)motor_get_back_off_steps(i));
        snprintf(key, sizeof(key), "hofs%d",    i); nvs_set_u32(h, key, (uint32_t)motor_get_home_offset_steps(i));
        snprintf(key, sizeof(key), "hdir%d",    i); nvs_set_u32(h, key, (uint32_t)(int32_t)motor_get_home_dir(i));
        snprintf(key, sizeof(key), "encdir%d",  i); nvs_set_u32(h, key, (uint32_t)(int32_t)motor_get_enc_dir(i));
        snprintf(key, sizeof(key), "gearratio%d", i);
        nvs_set_u32(h, key, float_to_u32(s_gear_ratio[i]));
    }
    nvs_set_u32(h, "microstep",    (uint32_t)s_microstep);
    nvs_set_u32(h, "idle_timeout", s_idle_timeout_ms);
    nvs_set_u32(h, "comm_timeout", s_comm_timeout_ms);

    esp_err_t err = nvs_commit(h);
    nvs_close(h);

    bool gear_ok = save_gear_config();
    if (err == ESP_OK && gear_ok) {
        ESP_LOGI(TAG, "Config saved to NVS");
        return true;
    }
    ESP_LOGE(TAG, "NVS commit failed: motor=%s gear=%s",
             esp_err_to_name(err), gear_ok ? "OK" : "FAIL");
    return false;
}

/* ------------------------------------------------------------------ */
/*  config_reset — デフォルト値に戻して NVS をクリア                     */
/* ------------------------------------------------------------------ */
void config_reset(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    if (nvs_open(GEAR_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }

    s_microstep       = MICROSTEP_32;
    s_idle_timeout_ms = DEF_IDLE_TIMEOUT_MS;
    s_comm_timeout_ms = DEF_COMM_TIMEOUT_MS;

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        s_profile[i] = DEF_PROFILE_PER_AXIS[i];
        const motor_profile_t *p = motor_spec_get_profile(s_profile[i]);
        if (p) apply_profile_params(i, p);
        motor_set_enc_dir(i, 1);
        s_gear_offset[i] = 0.0f;
        s_gear_dir[i] = 1;
        s_gear_abs_capable[i] = false;
        s_gear_enable[i] = true;
        s_gear_ratio[i] = 1.0f;
        if (p && p->encoder_counts_per_rev > 0) {
            uint32_t mpc = ((uint32_t)p->full_steps_per_rev * (uint32_t)MICROSTEP_32 * 100U)
                           / p->encoder_counts_per_rev;
            motor_set_mpc_x100(i, (uint16_t)(mpc > 0xFFFFU ? 0xFFFFU : mpc));
        }
    }
    apply_microstep_gpio(s_microstep);
    motor_set_idle_timeout(s_idle_timeout_ms);
    s_gear_deviation_warn = DEF_GEAR_DEVIATION_WARN_DEG;

    ESP_LOGI(TAG, "Config reset to defaults");
}

/* ------------------------------------------------------------------ */
/*  マイクロステップ                                                     */
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
    /* マイクロステップ変更に伴い mpc_x100 を全軸更新 */
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        const motor_profile_t *p = motor_spec_get_profile(s_profile[i]);
        if (p && p->encoder_counts_per_rev > 0) {
            uint32_t mpc = ((uint32_t)p->full_steps_per_rev * (uint32_t)div * 100U)
                           / p->encoder_counts_per_rev;
            motor_set_mpc_x100(i, (uint16_t)(mpc > 0xFFFFU ? 0xFFFFU : mpc));
        }
    }
    return true;
}

microstep_t config_get_microstep(void) { return s_microstep; }

/* ------------------------------------------------------------------ */
/*  アイドルタイムアウト                                                 */
/* ------------------------------------------------------------------ */
void     config_set_idle_timeout(uint32_t ms) { s_idle_timeout_ms = ms; }
uint32_t config_get_idle_timeout(void)        { return s_idle_timeout_ms; }

/* ------------------------------------------------------------------ */
/*  通信ウォッチドッグタイムアウト                                        */
/* ------------------------------------------------------------------ */
void     config_set_comm_timeout(uint32_t ms) { s_comm_timeout_ms = ms; }
uint32_t config_get_comm_timeout(void)        { return s_comm_timeout_ms; }

/* ------------------------------------------------------------------ */
/*  モータープロファイル                                                  */
/* ------------------------------------------------------------------ */
bool config_set_motor_profile(uint8_t axis, motor_profile_id_t id)
{
    if (axis >= NUM_AXES) return false;

    const motor_profile_t *p = motor_spec_get_profile(id);
    if (!p) return false;

    s_profile[axis] = id;
    apply_profile_params(axis, p);
    if (p->encoder_counts_per_rev > 0) {
        uint32_t mpc = ((uint32_t)p->full_steps_per_rev * (uint32_t)s_microstep * 100U)
                       / p->encoder_counts_per_rev;
        motor_set_mpc_x100(axis, (uint16_t)(mpc > 0xFFFFU ? 0xFFFFU : mpc));
    }

    /* NVS へ即時保存 (プロファイルと連動パラメータを同時に書く) */
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        char key[16];
        snprintf(key, sizeof(key), "mprof%d", axis);
        nvs_set_u32(h, key, (uint32_t)id);
        snprintf(key, sizeof(key), "vmax%d",  axis);
        nvs_set_u32(h, key, p->vmax_default);
        snprintf(key, sizeof(key), "accel%d", axis);
        nvs_set_u32(h, key, p->accel_default);
        snprintf(key, sizeof(key), "decel%d", axis);
        nvs_set_u32(h, key, p->decel_default);
        nvs_commit(h);
        nvs_close(h);
    }

    ESP_LOGI(TAG, "Axis %d motor profile → %s", axis, p->model);
    return true;
}

motor_profile_id_t config_get_motor_profile(uint8_t axis)
{
    if (axis >= NUM_AXES) return MOTOR_PROFILE_NONE;
    return s_profile[axis];
}

const motor_profile_t *config_get_motor_profile_data(uint8_t axis)
{
    if (axis >= NUM_AXES) return NULL;
    return motor_spec_get_profile(s_profile[axis]);
}

/* ------------------------------------------------------------------ */
/*  ギア出力角度モニタ設定                                                */
/* ------------------------------------------------------------------ */
float config_get_gear_offset(uint8_t axis)
{
    return axis < NUM_AXES ? s_gear_offset[axis] : 0.0f;
}

int8_t config_get_gear_dir(uint8_t axis)
{
    return axis < NUM_AXES ? s_gear_dir[axis] : 1;
}

bool config_get_gear_abs_capable(uint8_t axis)
{
    return axis < NUM_AXES && s_gear_abs_capable[axis];
}

bool config_get_gear_enable(uint8_t axis)
{
    return axis < NUM_AXES && s_gear_enable[axis];
}

float config_get_gear_deviation_warn(void)
{
    return s_gear_deviation_warn;
}

float config_get_gear_ratio(uint8_t axis)
{
    return axis < NUM_AXES ? s_gear_ratio[axis] : 1.0f;
}

bool config_set_gear_offset(uint8_t axis, float deg)
{
    if (axis >= NUM_AXES || !isfinite(deg)) return false;
    s_gear_offset[axis] = deg;
    return save_gear_config();
}

bool config_set_gear_dir(uint8_t axis, int8_t dir)
{
    if (axis >= NUM_AXES || (dir != 1 && dir != -1)) return false;
    s_gear_dir[axis] = dir;
    return save_gear_config();
}

bool config_set_gear_abs_capable(uint8_t axis, bool capable)
{
    if (axis >= NUM_AXES) return false;
    s_gear_abs_capable[axis] = capable;
    return save_gear_config();
}

bool config_set_gear_enable(uint8_t axis, bool enable)
{
    if (axis >= NUM_AXES) return false;
    s_gear_enable[axis] = enable;
    return save_gear_config();
}

bool config_set_gear_deviation_warn(float deg)
{
    if (!isfinite(deg) || deg < 0.0f) return false;
    s_gear_deviation_warn = deg;
    return save_gear_config();
}
