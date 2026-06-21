/* board_test.c — SteppingMotorDriver PCB 導通チェックツール
 *
 * USB-CDC シリアル接続後にコマンドで各ポートの入出力状態を確認する。
 * idf.py build flash でビルド・書き込み後、idf.py monitor で操作。
 * "help" コマンドでコマンド一覧を表示。
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

/* ================================================================
 *  ピン定義
 * ================================================================ */

/* 出力 GPIO — DRV8825 制御 */
#define PIN_STEP0       6
#define PIN_DIR0        7
#define PIN_STEP1       8
#define PIN_DIR1        9
#define PIN_STEP2      10
#define PIN_DIR2       11
#define PIN_DRV_EN     12   /* アクティブ Low  (1=非励磁, 0=励磁) */
#define PIN_DRV_RESET  13   /* アクティブ Low  (0=リセット保持)   */
#define PIN_DRV_SLEEP  14   /* アクティブ High (0=スリープ)        */
#define PIN_M0         41
#define PIN_M1         42
#define PIN_M2         45

/* 入力 GPIO — エンコーダ (AM26LV32 差動受信後の CMOS 出力) */
#define PIN_ENC0_A     15
#define PIN_ENC0_B     16
#define PIN_ENC0_Z     17
#define PIN_ENC1_A     18
#define PIN_ENC1_B     21
#define PIN_ENC1_Z     35
#define PIN_ENC2_A     36
#define PIN_ENC2_B     37
#define PIN_ENC2_Z     40

/* I2C 拡張バス */
#define PIN_I2C_SDA    38
#define PIN_I2C_SCL    39

/* ADC — ADC1 のみ使用 */
#define ADC_CH_POT0    ADC_CHANNEL_0  /* GPIO1  — ポテンショメーター ch0 */
#define ADC_CH_POT1    ADC_CHANNEL_1  /* GPIO2  — ポテンショメーター ch1 */
#define ADC_CH_POT2    ADC_CHANNEL_2  /* GPIO3  — ポテンショメーター ch2 */
#define ADC_CH_MOT_V   ADC_CHANNEL_3  /* GPIO4  — 24V 電源電圧モニタ    */
#define ADC_CH_CURRENT ADC_CHANNEL_4  /* GPIO5  — 電流センス             */

/* ADC 換算定数 */
#define ADC_VREF_MV        3300.0f
#define ADC_MAX_RAW        4095.0f
#define MOT_V_DIVIDER      8.0f   /* 24V→3V 分圧比 (回路実測値で要調整) */
#define CURRENT_GAIN_SCALE 2.0f   /* I[mA] = V_adc[mV] / 2 */

/* ================================================================
 *  出力 GPIO テーブル
 * ================================================================ */
typedef struct {
    const char *name;
    int         gpio;
    int         safe_level; /* 安全初期値 (0=Low, 1=High) */
} output_pin_t;

static const output_pin_t OUT_PINS[] = {
    { "STEP0",     PIN_STEP0,     0 },
    { "DIR0",      PIN_DIR0,      0 },
    { "STEP1",     PIN_STEP1,     0 },
    { "DIR1",      PIN_DIR1,      0 },
    { "STEP2",     PIN_STEP2,     0 },
    { "DIR2",      PIN_DIR2,      0 },
    { "DRV_EN",    PIN_DRV_EN,    1 }, /* High=非励磁 (安全側) */
    { "DRV_RESET", PIN_DRV_RESET, 0 }, /* Low=リセット保持      */
    { "DRV_SLEEP", PIN_DRV_SLEEP, 0 }, /* Low=スリープ          */
    { "M0",        PIN_M0,        1 }, /* 1/32 step デフォルト  */
    { "M1",        PIN_M1,        0 },
    { "M2",        PIN_M2,        1 },
};
#define N_OUT  ((int)(sizeof(OUT_PINS) / sizeof(OUT_PINS[0])))

/* ================================================================
 *  入力 GPIO テーブル
 * ================================================================ */
typedef struct {
    const char *name;
    int         gpio;
    int         ch;  /* エンコーダ ch (0–2) */
} input_pin_t;

static const input_pin_t IN_PINS[] = {
    { "ENC0_A", PIN_ENC0_A, 0 },
    { "ENC0_B", PIN_ENC0_B, 0 },
    { "ENC0_Z", PIN_ENC0_Z, 0 },
    { "ENC1_A", PIN_ENC1_A, 1 },
    { "ENC1_B", PIN_ENC1_B, 1 },
    { "ENC1_Z", PIN_ENC1_Z, 1 },
    { "ENC2_A", PIN_ENC2_A, 2 },
    { "ENC2_B", PIN_ENC2_B, 2 },
    { "ENC2_Z", PIN_ENC2_Z, 2 },
};
#define N_IN  ((int)(sizeof(IN_PINS) / sizeof(IN_PINS[0])))

/* ================================================================
 *  ADC テーブル
 * ================================================================ */
typedef struct {
    const char   *name;
    adc_channel_t ch;
    int            gpio;
} adc_pin_t;

static const adc_pin_t ADC_PINS[] = {
    { "POT0",    ADC_CH_POT0,    1 },
    { "POT1",    ADC_CH_POT1,    2 },
    { "POT2",    ADC_CH_POT2,    3 },
    { "MOT_V",   ADC_CH_MOT_V,   4 },
    { "CURRENT", ADC_CH_CURRENT, 5 },
};
#define N_ADC  ((int)(sizeof(ADC_PINS) / sizeof(ADC_PINS[0])))

static adc_oneshot_unit_handle_t s_adc;

/* ================================================================
 *  初期化
 * ================================================================ */
static void init_gpio(void)
{
    /* 出力ピン一括設定 */
    uint64_t out_mask = 0;
    for (int i = 0; i < N_OUT; i++) {
        out_mask |= (1ULL << OUT_PINS[i].gpio);
    }
    gpio_config_t cfg_out = {
        .pin_bit_mask = out_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_out);

    /* 安全初期値を設定 */
    for (int i = 0; i < N_OUT; i++) {
        gpio_set_level(OUT_PINS[i].gpio, OUT_PINS[i].safe_level);
    }

    /* 入力ピン — プルダウン (AM26LV32 未接続時に Low になる) */
    uint64_t in_mask = 0;
    for (int i = 0; i < N_IN; i++) {
        in_mask |= (1ULL << IN_PINS[i].gpio);
    }
    gpio_config_t cfg_in = {
        .pin_bit_mask = in_mask,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg_in);

    printf("[GPIO] 初期化完了\n");
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t u_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&u_cfg, &s_adc));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,  /* 0–3.3V レンジ */
    };
    for (int i = 0; i < N_ADC; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_PINS[i].ch, &ch_cfg));
    }
    printf("[ADC]  初期化完了\n");
}

/* ================================================================
 *  コマンド実装
 * ================================================================ */

/* safe — 全出力ピンを安全初期値に戻す */
static void cmd_safe(void)
{
    for (int i = 0; i < N_OUT; i++) {
        gpio_set_level(OUT_PINS[i].gpio, OUT_PINS[i].safe_level);
    }
    printf("[SAFE] 全出力ピンを安全初期値に設定しました\n");
    printf("       DRV_EN=1(非励磁) DRV_RESET=0(RST保持) DRV_SLEEP=0(SLP)\n");
    printf("       STEP×3=0 DIR×3=0 M0=1 M1=0 M2=1 (1/32step)\n");
}

/* out <pin> <0|1> — 出力ピンを指定値に設定 */
static void cmd_out(const char *name_s, const char *val_s)
{
    if (!name_s || !val_s) {
        printf("[ERR] 使い方: out <ピン名またはGPIO番号> <0|1>\n");
        return;
    }
    int level = atoi(val_s);
    if (level != 0 && level != 1) {
        printf("[ERR] 値は 0 または 1 のみ有効です\n");
        return;
    }
    /* 名前で検索 */
    for (int i = 0; i < N_OUT; i++) {
        if (strcasecmp(OUT_PINS[i].name, name_s) == 0) {
            gpio_set_level(OUT_PINS[i].gpio, level);
            printf("[OUT]  %-12s (GPIO%2d) = %d\n",
                   OUT_PINS[i].name, OUT_PINS[i].gpio, level);
            return;
        }
    }
    /* GPIO 番号で検索 */
    int gpio_num = atoi(name_s);
    for (int i = 0; i < N_OUT; i++) {
        if (OUT_PINS[i].gpio == gpio_num) {
            gpio_set_level(OUT_PINS[i].gpio, level);
            printf("[OUT]  %-12s (GPIO%2d) = %d\n",
                   OUT_PINS[i].name, OUT_PINS[i].gpio, level);
            return;
        }
    }
    printf("[ERR] 出力ピン '%s' が見つかりません\n", name_s);
}

/* in — 全エンコーダ入力ピンを読み取る */
static void cmd_in(void)
{
    printf("[IN]   エンコーダ入力ピン:\n");
    printf("       Ch  信号      GPIO   値\n");
    for (int i = 0; i < N_IN; i++) {
        printf("       %d   %-8s  GPIO%-2d  %s\n",
               IN_PINS[i].ch,
               IN_PINS[i].name,
               IN_PINS[i].gpio,
               gpio_get_level(IN_PINS[i].gpio) ? "HIGH" : "LOW ");
    }
}

/* adc — 全 ADC チャンネルを読み取る */
static void cmd_adc(void)
{
    printf("[ADC]  チャンネル読み取り (4回平均):\n");
    printf("       名前      GPIO  RAW    電圧[mV]  換算値\n");

    for (int i = 0; i < N_ADC; i++) {
        int sum = 0;
        for (int j = 0; j < 4; j++) {
            int raw = 0;
            adc_oneshot_read(s_adc, ADC_PINS[i].ch, &raw);
            sum += raw;
        }
        int  raw_avg = sum / 4;
        float v_mv   = (float)raw_avg * ADC_VREF_MV / ADC_MAX_RAW;

        char conv[40] = "";
        if (i < 3) {
            /* POT0–2: ポテンショメーター → 0–100% */
            snprintf(conv, sizeof(conv), "%.1f%%", v_mv / ADC_VREF_MV * 100.0f);
        } else if (i == 3) {
            /* MOT_V: 分圧比から電源電圧を換算 */
            snprintf(conv, sizeof(conv), "%.2fV (24V系)", v_mv / 1000.0f * MOT_V_DIVIDER);
        } else {
            /* CURRENT: 電流換算 (I[mA] = V[mV] / 2) */
            snprintf(conv, sizeof(conv), "%.1fmA", v_mv / CURRENT_GAIN_SCALE);
        }

        printf("       %-8s  GPIO%-2d  %-6d %-9.1f %s\n",
               ADC_PINS[i].name, ADC_PINS[i].gpio, raw_avg, v_mv, conv);
    }
}

/* i2c — I2C バスをスキャンしてデバイスを検出 */
static void cmd_i2c(void)
{
    printf("[I2C]  バススキャン (SDA=GPIO%d / SCL=GPIO%d, 100kHz) ...\n",
           PIN_I2C_SDA, PIN_I2C_SCL);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port                   = I2C_NUM_0,
        .sda_io_num                 = PIN_I2C_SDA,
        .scl_io_num                 = PIN_I2C_SCL,
        .clk_source                 = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt          = 7,
        .flags.enable_internal_pullup = false, /* 基板上に外部プルアップあり */
    };
    i2c_master_bus_handle_t bus;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        printf("[ERR] I2C バス初期化失敗: %s\n", esp_err_to_name(err));
        return;
    }

    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(bus, addr, 10) == ESP_OK) {
            printf("       デバイス検出: 0x%02X\n", addr);
            found++;
        }
    }
    printf("       スキャン完了 — ");
    if (found == 0) {
        printf("デバイスなし\n");
    } else {
        printf("%d 個のデバイスを検出\n", found);
    }

    i2c_del_master_bus(bus);
}

/* walk [ms] — 出力ピンを1本ずつ安全値の反転方向に動かして導通確認 */
static void cmd_walk(int dwell_ms)
{
    if (dwell_ms <= 0) dwell_ms = 500;

    printf("[WALK] 各出力ピンを %dms ずつ切り替えます\n", dwell_ms);
    printf("       テスターまたはオシロスコープで各ピン波形を確認してください\n\n");

    cmd_safe();
    vTaskDelay(pdMS_TO_TICKS(300));

    for (int i = 0; i < N_OUT; i++) {
        int test_level = 1 - OUT_PINS[i].safe_level; /* 安全値の反転 */

        /* DRV_EN を Low にするとモーター励磁が発生 — 警告 */
        if (OUT_PINS[i].gpio == PIN_DRV_EN && test_level == 0) {
            printf("  [%2d/%2d] %-12s (GPIO%2d) [!] Low にすると励磁発生\n",
                   i + 1, N_OUT, OUT_PINS[i].name, OUT_PINS[i].gpio);
        }

        gpio_set_level(OUT_PINS[i].gpio, test_level);
        printf("  [%2d/%2d] %-12s (GPIO%2d) safe=%d → test=%d (測定中...)\n",
               i + 1, N_OUT,
               OUT_PINS[i].name, OUT_PINS[i].gpio,
               OUT_PINS[i].safe_level, test_level);

        vTaskDelay(pdMS_TO_TICKS(dwell_ms));

        gpio_set_level(OUT_PINS[i].gpio, OUT_PINS[i].safe_level);
        printf("          %-12s (GPIO%2d) → safe=%d\n",
               OUT_PINS[i].name, OUT_PINS[i].gpio, OUT_PINS[i].safe_level);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    printf("\n[WALK] 完了 — 全ピンを安全値に復帰\n");
}

/* all <0|1> — 全出力ピンを一括設定 */
static void cmd_all(int level)
{
    for (int i = 0; i < N_OUT; i++) {
        gpio_set_level(OUT_PINS[i].gpio, level);
    }
    printf("[ALL]  全出力ピンを %d に設定しました\n", level);
}

/* step <ch> <count> [hz] — テスト用ステップパルス生成 (DRV_EN=High のまま) */
static void cmd_step(int ch, int count, int freq_hz)
{
    if (ch < 0 || ch > 2) {
        printf("[ERR] ch は 0–2 の範囲で指定してください\n");
        return;
    }
    if (count <= 0)  count   = 100;
    if (freq_hz <= 0) freq_hz = 1000;

    const int gpio_step = (ch == 0) ? PIN_STEP0 : (ch == 1) ? PIN_STEP1 : PIN_STEP2;
    const int gpio_dir  = (ch == 0) ? PIN_DIR0  : (ch == 1) ? PIN_DIR1  : PIN_DIR2;

    /* DRV8825 最小パルス幅 1.9µs を保証するため half_period >= 2µs */
    int half_us = 1000000 / (freq_hz * 2);
    if (half_us < 2) half_us = 2;

    printf("[STEP] CH%d  GPIO%d  %d パルス  %d Hz  half=%dµs\n",
           ch, gpio_step, count, freq_hz, half_us);
    printf("       DIR (GPIO%d) = 0 (正転)  DRV_EN=1 (励磁なし)\n", gpio_dir);

    gpio_set_level(gpio_dir, 0);
    esp_rom_delay_us(2);  /* DIR セットアップ時間 (DRV8825: 650ns 以上) */

    for (int i = 0; i < count; i++) {
        gpio_set_level(gpio_step, 1);
        esp_rom_delay_us(half_us);
        gpio_set_level(gpio_step, 0);
        esp_rom_delay_us(half_us);
    }

    printf("[STEP] %d パルス送出完了 (DRV_EN=High のため実際のモーター動作なし)\n",
           count);
}

/* status — 全ピン・ADC の状態を一覧表示 */
static void cmd_status(void)
{
    printf("\n======== SteppingMotorDriver ボード状態 ========\n\n");

    printf("  出力ピン:\n");
    printf("    %-12s  GPIO  現在値  安全値\n", "名前");
    for (int i = 0; i < N_OUT; i++) {
        int v = gpio_get_level(OUT_PINS[i].gpio);
        printf("    %-12s  %4d  %-6s  %d\n",
               OUT_PINS[i].name, OUT_PINS[i].gpio,
               v ? "HIGH" : "LOW ",
               OUT_PINS[i].safe_level);
    }

    printf("\n  入力ピン:\n");
    cmd_in();

    printf("\n  ADC:\n");
    cmd_adc();

    printf("\n================================================\n\n");
}

/* ================================================================
 *  ヘルプ
 * ================================================================ */
static void cmd_help(void)
{
    printf("\n======= SteppingMotorDriver 導通チェックツール =======\n\n");
    printf("コマンド一覧:\n");
    printf("  help              このヘルプを表示\n");
    printf("  status            全ピン・ADC 状態を一覧表示\n");
    printf("  safe              全出力ピンを安全初期値に戻す\n");
    printf("  out <pin> <0|1>   出力ピンを設定 (pin: 名前 or GPIO番号)\n");
    printf("  in                全エンコーダ入力ピンを読み取る\n");
    printf("  adc               全 ADC チャンネルを読み取る\n");
    printf("  i2c               I2C バスをスキャン (要: 外部プルアップ)\n");
    printf("  walk [ms]         出力ピンを順番に切り替えて導通確認 (デフォルト 500ms)\n");
    printf("  all <0|1>         全出力ピンを一括設定\n");
    printf("  step <ch> <n> [hz] CH0-2 にステップパルスを生成 (DRV_EN=High のまま)\n");
    printf("\n出力ピン名 (out / walk コマンドで使用):\n");
    for (int i = 0; i < N_OUT; i++) {
        printf("  %-12s  GPIO%-2d  (安全値=%d)\n",
               OUT_PINS[i].name, OUT_PINS[i].gpio, OUT_PINS[i].safe_level);
    }
    printf("\n入力ピン名:\n");
    for (int i = 0; i < N_IN; i++) {
        printf("  %-10s  GPIO%-2d\n", IN_PINS[i].name, IN_PINS[i].gpio);
    }
    printf("\n注意:\n");
    printf("  - DRV_EN=1 (High) の間、モーター励磁はありません (安全)\n");
    printf("  - 'out DRV_EN 0' でコイル励磁が発生します\n");
    printf("  - エンコーダ入力は AM26LV32 経由のため、\n");
    printf("    差動入力が接続されていない場合は Low になります\n");
    printf("  - ADC の MOT_V 換算は分圧比 %.1f を使用しています\n", MOT_V_DIVIDER);
    printf("    (実際の回路値で要調整: MOT_V_DIVIDER 定数)\n\n");
}

/* ================================================================
 *  コマンドパーサ
 * ================================================================ */
static void process_line(char *line)
{
    /* 先頭・末尾ホワイトスペース除去 */
    while (isspace((unsigned char)*line)) line++;
    char *end = line + strlen(line) - 1;
    while (end >= line && isspace((unsigned char)*end)) { *end = '\0'; end--; }
    if (strlen(line) == 0) return;

    /* トークン分割 */
    char *tok[8] = {0};
    int ntok = 0;
    char *p = line;
    while (*p && ntok < 8) {
        while (isspace((unsigned char)*p)) p++;
        if (!*p) break;
        tok[ntok++] = p;
        while (*p && !isspace((unsigned char)*p)) p++;
        if (*p) *p++ = '\0';
    }
    if (ntok == 0) return;

    /* コマンド振り分け */
    if      (strcasecmp(tok[0], "help")   == 0) { cmd_help(); }
    else if (strcasecmp(tok[0], "status") == 0) { cmd_status(); }
    else if (strcasecmp(tok[0], "safe")   == 0) { cmd_safe(); }
    else if (strcasecmp(tok[0], "out")    == 0) { cmd_out(tok[1], tok[2]); }
    else if (strcasecmp(tok[0], "in")     == 0) { cmd_in(); }
    else if (strcasecmp(tok[0], "adc")    == 0) { cmd_adc(); }
    else if (strcasecmp(tok[0], "i2c")    == 0) { cmd_i2c(); }
    else if (strcasecmp(tok[0], "walk")   == 0) {
        cmd_walk(ntok > 1 ? atoi(tok[1]) : 500);
    }
    else if (strcasecmp(tok[0], "all")    == 0) {
        if (ntok < 2) { printf("[ERR] 使い方: all <0|1>\n"); }
        else { cmd_all(atoi(tok[1])); }
    }
    else if (strcasecmp(tok[0], "step")   == 0) {
        cmd_step(ntok > 1 ? atoi(tok[1]) : 0,
                 ntok > 2 ? atoi(tok[2]) : 100,
                 ntok > 3 ? atoi(tok[3]) : 1000);
    }
    else {
        printf("[ERR] 不明なコマンド: '%s'  (help でコマンド一覧)\n", tok[0]);
    }
}

/* ================================================================
 *  app_main
 * ================================================================ */
void app_main(void)
{
    /* USB CDC が安定するまで少し待機 */
    vTaskDelay(pdMS_TO_TICKS(500));

    printf("\n");
    printf("+------------------------------------------+\n");
    printf("|  SteppingMotorDriver  PCB 導通チェック  |\n");
    printf("+------------------------------------------+\n\n");

    init_gpio();
    init_adc();

    printf("\n起動完了。'help' でコマンド一覧を表示します。\n\n");
    printf("> ");
    fflush(stdout);

    /* fgets は USB Serial/JTAG で1文字ずつ返るため使用しない。
     * fgetc でエコーバックしながら1行ずつ蓄積する。 */
    char line[128];
    int  len = 0;

    while (true) {
        int c = fgetc(stdin);
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (c == '\r' || c == '\n') {
            /* 改行: 行処理 */
            printf("\n");
            fflush(stdout);
            if (len > 0) {
                line[len] = '\0';
                process_line(line);
                len = 0;
            }
            printf("> ");
            fflush(stdout);
        } else if (c == '\b' || c == 0x7f) {
            /* バックスペース */
            if (len > 0) {
                len--;
                printf("\b \b");
                fflush(stdout);
            }
        } else if (len < (int)(sizeof(line) - 1)) {
            /* 通常文字: バッファに追加してエコー */
            line[len++] = (char)c;
            fputc(c, stdout);
            fflush(stdout);
        }
    }
}
