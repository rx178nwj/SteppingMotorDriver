#include "encoder.h"
#include "gpio_config.h"
#include "motor_ctrl.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "encoder";

/* ====================================================================
   定数
   ==================================================================== */

/* PCNT カウンタ範囲: [-PCNT_HIGH, +PCNT_HIGH]
   この値に達したらオーバーフロー ISR が pos_accum を補正し、
   カウンタが 0 にリセットされる。
   モーター最高速 96000 µsteps/s @ 1/32、エンコーダ 4000 counts/rev、
   200 steps/rev → 最大 96000/32 × 4000/200 = 60000 counts/s。
   10ms ループで最大 600 counts/周期 → 16000 は十分なマージン。   */
#define PCNT_HIGH_LIMIT    16000

/* 200ms 以上カウント変化がない場合に速度 = 0 と判定 */
#define VEL_ZERO_US        200000LL

/* ====================================================================
   GPIO マッピング（軸番号でインデックス）
   ==================================================================== */
static const uint8_t k_gpio_a[NUM_AXES] = { GPIO_ENC0_A, GPIO_ENC1_A, GPIO_ENC2_A };
static const uint8_t k_gpio_b[NUM_AXES] = { GPIO_ENC0_B, GPIO_ENC1_B, GPIO_ENC2_B };
static const uint8_t k_gpio_z[NUM_AXES] = { GPIO_ENC0_Z, GPIO_ENC1_Z, GPIO_ENC2_Z };

/* ====================================================================
   軸ごとの内部状態
   ==================================================================== */
typedef struct {
    pcnt_unit_handle_t  unit;
    bool                enabled;       /* true = PCNT/Z相 GPIO 初期化済み (CLOSED_LOOP軸のみ) */

    /* 32bit 位置 = pos_accum + pcnt_raw_count
       pos_accum は PCNT オーバーフロー ISR のみが更新する。         */
    volatile int32_t    pos_accum;
    portMUX_TYPE        mux;           /* pos_accum / encoder_set_pos 排他制御 */

    /* 速度推定（10ms ループ用スナップショット） */
    int32_t             pos_snap;
    int64_t             t_snap_us;
    int64_t             last_move_us;  /* 最後にカウント変化した時刻 */
    int32_t             vel_cps;       /* 現在速度 [counts/sec] */

    /* Z 相インデックスイベント */
    volatile bool       z_event;
} encoder_axis_t;

static encoder_axis_t s_ax[NUM_AXES];

/* ====================================================================
   PCNT オーバーフロー ISR コールバック

   ESP32-S3 PCNT はラップ値が常に 0 (TRM: "cleared to 0")。
   対称範囲 [-H, +H] でも +H → 0 / -H → 0 で折り返す。

   連続性条件: accum_new + 0 = accum_old + watch_point_val
              → Δaccum = watch_point_val (= ±PCNT_HIGH_LIMIT)

   注: ハードウェアリセット(raw→0)と本 ISR 発火の間に約 1〜5µs の
   窓があり、この間 encoder_get_pos() が誤値を返す可能性がある。
   発生頻度は最高速 60000cps で約 9 分に 1 回程度と見積もられるため、
   呼び出し側で突発スパイクのサニティチェックを行うこと。
   ==================================================================== */
static bool IRAM_ATTR pcnt_overflow_cb(pcnt_unit_handle_t unit,
                                        const pcnt_watch_event_data_t *edata,
                                        void *ctx)
{
    encoder_axis_t *ax = (encoder_axis_t *)ctx;
    portENTER_CRITICAL_ISR(&ax->mux);
    if (edata->watch_point_value == PCNT_HIGH_LIMIT) {
        ax->pos_accum += (int32_t)PCNT_HIGH_LIMIT;
    } else if (edata->watch_point_value == -PCNT_HIGH_LIMIT) {
        ax->pos_accum -= (int32_t)PCNT_HIGH_LIMIT;
    }
    portEXIT_CRITICAL_ISR(&ax->mux);
    return false;   /* 高優先度タスクへの wake 不要 */
}

/* ====================================================================
   Z 相 GPIO ISR
   ==================================================================== */
static void IRAM_ATTR z_isr_handler(void *ctx)
{
    encoder_axis_t *ax = (encoder_axis_t *)ctx;
    ax->z_event = true;
}

/* ====================================================================
   1 軸分 PCNT 初期化
   4 逓倍 A/B 直交デコード:
     Ch0: A エッジ / B レベル  → B=High 時に符号反転
     Ch1: B エッジ / A レベル  → A=Low  時に符号反転
   ==================================================================== */
static void init_pcnt_axis(uint8_t i)
{
    encoder_axis_t *ax = &s_ax[i];

    pcnt_unit_config_t unit_cfg = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit  = -PCNT_HIGH_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &ax->unit));

    /* AM26LV32 差動出力のグリッチ除去: 1µs 未満を無視 */
    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(ax->unit, &filter_cfg));

    /* チャンネル 0: A エッジ / B レベル */
    pcnt_chan_config_t cfg_a = {
        .edge_gpio_num  = k_gpio_a[i],
        .level_gpio_num = k_gpio_b[i],
    };
    pcnt_channel_handle_t chan_a;
    ESP_ERROR_CHECK(pcnt_new_channel(ax->unit, &cfg_a, &chan_a));
    pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   /* A 立上り → +1 */
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);  /* A 立下り → -1 */
    pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,   /* B=High → 符号反転 */
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);     /* B=Low  → そのまま */

    /* チャンネル 1: B エッジ / A レベル */
    pcnt_chan_config_t cfg_b = {
        .edge_gpio_num  = k_gpio_b[i],
        .level_gpio_num = k_gpio_a[i],
    };
    pcnt_channel_handle_t chan_b;
    ESP_ERROR_CHECK(pcnt_new_channel(ax->unit, &cfg_b, &chan_b));
    pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,      /* A=High → そのまま */
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);  /* A=Low  → 符号反転 */

    /* オーバーフロー検出用ウォッチポイント登録 */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ax->unit,  PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ax->unit, -PCNT_HIGH_LIMIT));

    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_overflow_cb };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(ax->unit, &cbs, ax));

    ESP_ERROR_CHECK(pcnt_unit_enable(ax->unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(ax->unit));
    ESP_ERROR_CHECK(pcnt_unit_start(ax->unit));
}

/* ====================================================================
   1 軸分 Z 相 GPIO 初期化
   ==================================================================== */
static void init_z_gpio(uint8_t i)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << k_gpio_z[i]),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_isr_handler_add(k_gpio_z[i], z_isr_handler, &s_ax[i]));
}

/* ====================================================================
   公開 API
   ==================================================================== */

void encoder_init(void)
{
    memset(s_ax, 0, sizeof(s_ax));

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        portMUX_TYPE init_mux = portMUX_INITIALIZER_UNLOCKED;
        s_ax[i].mux = init_mux;
    }

    /* GPIO ISR サービスが未インストールなら登録（他モジュールが先行していれば無視） */
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

    int64_t now = esp_timer_get_time();
    for (uint8_t i = 0; i < NUM_AXES; i++) {
        if (motor_get_motor_type(i) == MOTOR_TYPE_OPEN_LOOP) {
            /* オープンループ軸: エンコーダ未配線を前提に PCNT/Z相 GPIO を
               初期化しない（未接続 GPIO のフローティングノイズを誤カウント
               しないようにするため）。encoder_get_pos() 等は 0 を返す。   */
            ESP_LOGI(TAG, "Axis %d: OPEN_LOOP — encoder init skipped", i);
            continue;
        }
        init_pcnt_axis(i);
        init_z_gpio(i);
        s_ax[i].enabled       = true;
        s_ax[i].t_snap_us    = now;
        s_ax[i].last_move_us = now;
    }

    ESP_LOGI(TAG, "encoder init done (3ch, 4x quadrature, PPR×4=4000 counts/rev)");
}

/* --------------------------------------------------------------------
   位置取得 — ロックフリーリトライ

   ISR が accum 読み取りと raw 読み取りの間に割り込んだ場合はリトライ。
   raw を前後2回読み、変化があればカウントが進んだ(またはラップが起きた)
   ため再試行することで、ISR が raw 読み取りの直後に割り込んだケースも捕捉する。

   既知の制限: ハードウェアラップ(raw→0)と ISR 発火の間の約 1〜5µs 窓では
   上記リトライでも誤値を返す可能性がある。消費側でのフィルタ推奨。
   -------------------------------------------------------------------- */
int32_t encoder_get_pos(uint8_t axis)
{
    if (axis >= NUM_AXES) return 0;
    encoder_axis_t *ax = &s_ax[axis];
    if (!ax->enabled) return 0;   /* OPEN_LOOP軸: PCNT未初期化 */
    int32_t accum;
    int     raw1, raw2;
    do {
        accum = ax->pos_accum;
        pcnt_unit_get_count(ax->unit, &raw1);
        pcnt_unit_get_count(ax->unit, &raw2);
    } while (accum != ax->pos_accum || raw1 != raw2);
    return accum + (int32_t)raw1;
}

/* --------------------------------------------------------------------
   位置セット — ホーミング完了時などに呼ぶ
   モーターが停止している状態で呼び出すこと。
   -------------------------------------------------------------------- */
void encoder_set_pos(uint8_t axis, int32_t pos)
{
    if (axis >= NUM_AXES) return;
    encoder_axis_t *ax = &s_ax[axis];
    if (!ax->enabled) return;   /* OPEN_LOOP軸: PCNT未初期化 */

    taskENTER_CRITICAL(&ax->mux);
    pcnt_unit_clear_count(ax->unit);   /* PCNT raw → 0 */
    ax->pos_accum = pos;               /* pos = pos_accum(=pos) + raw(=0) */
    taskEXIT_CRITICAL(&ax->mux);

    int64_t now      = esp_timer_get_time();
    ax->pos_snap     = pos;
    ax->t_snap_us    = now;
    ax->last_move_us = now;
    ax->vel_cps      = 0;
}

int32_t encoder_get_vel_cps(uint8_t axis)
{
    if (axis >= NUM_AXES) return 0;
    return s_ax[axis].vel_cps;
}

bool encoder_take_z_event(uint8_t axis)
{
    if (axis >= NUM_AXES) return false;
    if (!s_ax[axis].z_event) return false;
    s_ax[axis].z_event = false;
    return true;
}

/* --------------------------------------------------------------------
   速度更新 — MotorControlTask の 10ms ループから呼ぶ

   高速域: vel = Δcount / Δt
   ゼロ判定: 最終移動から VEL_ZERO_US (200ms) 経過で vel = 0
   -------------------------------------------------------------------- */
void encoder_update_10ms(void)
{
    int64_t now_us = esp_timer_get_time();

    for (uint8_t i = 0; i < NUM_AXES; i++) {
        encoder_axis_t *ax = &s_ax[i];
        if (!ax->enabled) continue;   /* OPEN_LOOP軸: PCNT未初期化 */
        int32_t pos_now = encoder_get_pos(i);
        int64_t dt_us   = now_us - ax->t_snap_us;
        if (dt_us <= 0) continue;

        int32_t delta = pos_now - ax->pos_snap;
        if (delta != 0) {
            ax->last_move_us = now_us;
        }

        if ((now_us - ax->last_move_us) > VEL_ZERO_US) {
            ax->vel_cps = 0;
        } else {
            ax->vel_cps = (int32_t)((int64_t)delta * 1000000LL / dt_us);
        }

        ax->pos_snap  = pos_now;
        ax->t_snap_us = now_us;
    }
}
