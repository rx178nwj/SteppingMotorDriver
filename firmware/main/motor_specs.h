#pragma once
#include <stdint.h>

/* ====================================================================
   モータープロファイル定義
   設計前提: 24V 電源 / DRV8825 定電流チョッパ / 1/32 マイクロステップ
   ==================================================================== */

/* --------------------------------------------------------------------
   トルクカーブ節点
   full_steps_per_sec : マイクロステップ非依存の全ステップ換算速度
   torque_pct         : 保持トルクに対する割合 [%]
   -------------------------------------------------------------------- */
typedef struct {
    uint32_t full_steps_per_sec;
    uint8_t  torque_pct;
} motor_torque_point_t;

/* --------------------------------------------------------------------
   モータープロファイル ID
   NVS に uint32_t として保存されるため値を固定する。
   -------------------------------------------------------------------- */
typedef enum {
    MOTOR_PROFILE_NONE              = 0,
    MOTOR_PROFILE_17HS08_1004_ME1K  = 1,   /* CH0 デフォルト */
    MOTOR_PROFILE_17HS15_1504_ME1K  = 2,   /* CH1/CH2 デフォルト */
    MOTOR_PROFILE_COUNT
} motor_profile_id_t;

/* --------------------------------------------------------------------
   モータープロファイル構造体
   vmax_default / accel_default / decel_default は
   1/32 マイクロステップ・24V 駆動時の推奨値。
   -------------------------------------------------------------------- */
typedef struct {
    const char                 *model;
    uint16_t                    full_steps_per_rev;     /* steps/rev */
    uint16_t                    rated_current_ma;       /* 定格電流 [mA] */
    uint16_t                    phase_resistance_mohm;  /* 相抵抗 [mΩ] */
    uint16_t                    phase_inductance_uh;    /* 相インダクタンス [µH] */
    uint16_t                    holding_torque_mnm;     /* 保持トルク [mN·m] */
    uint8_t                     detent_torque_mnm;      /* ディテントトルク [mN·m] */
    uint16_t                    rotor_inertia_gcm2;     /* ロータ慣性 [g·cm²] */
    uint16_t                    motor_weight_g;         /* 質量 [g] */
    uint16_t                    time_const_us;          /* 電気時定数 τ=L/R [µs] */
    uint16_t                    current_rise_us_24v;    /* 電流立上り時間 @24V [µs] */
    uint16_t                    max_full_sps_full_torque; /* 全トルク維持の全ステップ上限 */
    /* 推奨動作パラメータ (1/32 マイクロステップ, 24V) */
    uint32_t                    vmax_default;           /* microsteps/sec */
    uint32_t                    accel_default;          /* microsteps/sec² */
    uint32_t                    decel_default;          /* microsteps/sec² */
    uint32_t                    vmax_hard_limit;        /* 絶対上限 microsteps/sec */
    /* トルクカーブ */
    const motor_torque_point_t *torque_curve;
    uint8_t                     torque_curve_len;
    /* ------------------------------------------------------------
       エンコーダー仕様 (ME1K 共通: インクリメンタル光学式, 差動出力)
       counts_per_rev = encoder_ppr × 4  (PCNT 4逓倍後の実効分解能)

       マイクロステップ比率 (1/32 時):
         microsteps_per_rev = full_steps_per_rev × 32 = 6400
         counts_per_rev     = encoder_ppr × 4        = 4000
         → 1 count ≈ 1.60 microsteps
            1 microstep ≈ 0.625 counts

       脱調検出閾値の目安: |Δpos_cmd - Δenc_count × 1.60| > 閾値
       ------------------------------------------------------------ */
    uint16_t                    encoder_ppr;            /* ライン数 [lines/rev] */
    uint16_t                    encoder_counts_per_rev; /* PCNT 4逓倍後 [counts/rev] */
    /* microsteps_per_count_x100: (microsteps_per_rev / counts_per_rev) × 100
       整数演算用スケーリング済み比率。1/32 時: 6400/4000×100 = 160             */
    uint16_t                    microsteps_per_count_x100;
} motor_profile_t;

/* --------------------------------------------------------------------
   チャンネルデフォルトプロファイル ID
   -------------------------------------------------------------------- */
#define MOTOR_CH0_DEFAULT_PROFILE   MOTOR_PROFILE_17HS08_1004_ME1K
#define MOTOR_CH1_DEFAULT_PROFILE   MOTOR_PROFILE_17HS15_1504_ME1K
#define MOTOR_CH2_DEFAULT_PROFILE   MOTOR_PROFILE_17HS15_1504_ME1K

/* --------------------------------------------------------------------
   API
   -------------------------------------------------------------------- */

/* プロファイル取得。id が範囲外または NONE の場合は NULL を返す。 */
const motor_profile_t *motor_spec_get_profile(motor_profile_id_t id);

/* 登録済みプロファイル数 (NONE を除く) */
uint8_t motor_spec_profile_count(void);

/* 指定速度における保持トルク比率 [%] を区間線形補間で返す。
   microstep_div: 分割数 (1/32 → 32)
   microsteps_per_sec: 現在速度                                         */
uint8_t motor_spec_torque_pct(const motor_torque_point_t *curve,
                               uint8_t                     len,
                               uint32_t                    microstep_div,
                               uint32_t                    microsteps_per_sec);

/* エンコーダー比率をランタイムのマイクロステップ設定で再計算する。
   戻り値: microsteps_per_count × 100 (整数演算用スケール済み)
   例: 1/32 → (200×32)/4000×100 = 160
       1/16 → (200×16)/4000×100 = 80
   プロファイルの .microsteps_per_count_x100 は 1/32 固定値。
   実行時のマイクロステップが異なる場合はこの関数で計算する。         */
uint16_t motor_spec_enc_ratio_x100(const motor_profile_t *p,
                                    uint16_t               microstep_div);
