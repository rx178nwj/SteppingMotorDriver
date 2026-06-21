#include "motor_specs.h"

/* ====================================================================
   トルクカーブデータ (24V, DRV8825 定電流モード)
   full_steps_per_sec: マイクロステップ非依存の全ステップ換算速度
   torque_pct        : 保持トルクに対する割合 [%]
   実機測定後に節点値を更新してキャリブレーションすること。
   ==================================================================== */

/* 17HS08-1004-ME1K (1.0A, 保持トルク 120mN·m) */
static const motor_torque_point_t s_curve_17hs08[] = {
    {    0, 100},   /*    0 rpm :  120 mN·m (100%) */
    {  400,  88},   /*  120 rpm :  106 mN·m ( 88%) */
    {  800,  71},   /*  240 rpm :   85 mN·m ( 71%) — v_max 推奨上限 */
    { 1200,  50},   /*  360 rpm :   60 mN·m ( 50%) */
    { 2000,  25},   /*  600 rpm :   30 mN·m ( 25%) */
    { 3000,   8},   /*  900 rpm :   10 mN·m (  8%) — pull-out 近傍 */
};

/* 17HS15-1504-ME1K (1.5A, 保持トルク 400mN·m) */
static const motor_torque_point_t s_curve_17hs15[] = {
    {    0, 100},   /*    0 rpm :  400 mN·m (100%) */
    {  500,  88},   /*  150 rpm :  352 mN·m ( 88%) */
    { 1000,  75},   /*  300 rpm :  300 mN·m ( 75%) — v_max 推奨上限 */
    { 2000,  50},   /*  600 rpm :  200 mN·m ( 50%) */
    { 3000,  20},   /*  900 rpm :   80 mN·m ( 20%) */
    { 4000,   5},   /* 1200 rpm :   20 mN·m (  5%) — pull-out 近傍 */
};

/* ====================================================================
   モータープロファイルテーブル
   インデックス = motor_profile_id_t の値 (NONE=0 はスキップ)
   ==================================================================== */
static const motor_profile_t s_profiles[MOTOR_PROFILE_COUNT] = {

    /* [0] MOTOR_PROFILE_NONE — プレースホルダー */
    [MOTOR_PROFILE_NONE] = { .model = NULL },

    /* ------------------------------------------------------------------
       [1] 17HS08-1004-ME1K
       NEMA17 (42.3mm), ボディ長 33mm, 2相バイポーラ, エンコーダ付
       τ = L/R = 3.5mH/2.5Ω = 1.4ms
       t_rise @24V = L×I/V = 3.5mH×1.0A/24V ≈ 146µs → 上限 ≈ 3425 steps/s
       v_max: 保持トルク 71% 維持 → 800 full steps/s × 32 = 25600 µsteps/s
       accel: T×10%/J → 150000 µsteps/s² (0.17s で v_max 到達)
       ------------------------------------------------------------------ */
    [MOTOR_PROFILE_17HS08_1004_ME1K] = {
        .model                        = "17HS08-1004-ME1K",
        .full_steps_per_rev           = 200,
        .rated_current_ma             = 1000,
        .phase_resistance_mohm        = 2500,
        .phase_inductance_uh          = 3500,
        .holding_torque_mnm           = 120,
        .detent_torque_mnm            = 7,
        .rotor_inertia_gcm2           = 35,
        .motor_weight_g               = 150,
        .time_const_us                = 1400,
        .current_rise_us_24v          = 146,
        .max_full_sps_full_torque     = 3425,
        .vmax_default                 = 25600,
        .accel_default                = 150000,
        .decel_default                = 150000,
        .vmax_hard_limit              = 80000,
        .torque_curve                 = s_curve_17hs08,
        .torque_curve_len             = (uint8_t)(sizeof(s_curve_17hs08) /
                                                  sizeof(s_curve_17hs08[0])),
        /* エンコーダー (ME1K: 1000 PPR, 差動出力, PCNT 4逓倍)
           counts_per_rev = 1000 × 4 = 4000
           microsteps_per_count @1/32 = (200×32)/4000 × 100 = 160            */
        .encoder_ppr                  = 1000,
        .encoder_counts_per_rev       = 4000,
        .microsteps_per_count_x100    = 160,
    },

    /* ------------------------------------------------------------------
       [2] 17HS15-1504-ME1K
       NEMA17 (42.3mm), ボディ長 40mm, 2相バイポーラ, エンコーダ付
       τ = L/R = 2.8mH/1.65Ω ≈ 1.7ms
       t_rise @24V = L×I/V = 2.8mH×1.5A/24V ≈ 175µs → 上限 ≈ 2857 steps/s
       v_max: 保持トルク 75% 維持 → 1000 full steps/s × 32 = 32000 µsteps/s
       accel: T×10%/J → 200000 µsteps/s² (0.16s で v_max 到達)
       ------------------------------------------------------------------ */
    [MOTOR_PROFILE_17HS15_1504_ME1K] = {
        .model                        = "17HS15-1504-ME1K",
        .full_steps_per_rev           = 200,
        .rated_current_ma             = 1500,
        .phase_resistance_mohm        = 1650,
        .phase_inductance_uh          = 2800,
        .holding_torque_mnm           = 400,
        .detent_torque_mnm            = 20,
        .rotor_inertia_gcm2           = 68,
        .motor_weight_g               = 280,
        .time_const_us                = 1697,
        .current_rise_us_24v          = 175,
        .max_full_sps_full_torque     = 2857,
        .vmax_default                 = 32000,
        .accel_default                = 200000,
        .decel_default                = 200000,
        .vmax_hard_limit              = 96000,
        .torque_curve                 = s_curve_17hs15,
        .torque_curve_len             = (uint8_t)(sizeof(s_curve_17hs15) /
                                                  sizeof(s_curve_17hs15[0])),
        /* エンコーダー (ME1K: 1000 PPR, 差動出力, PCNT 4逓倍)
           counts_per_rev = 1000 × 4 = 4000
           microsteps_per_count @1/32 = (200×32)/4000 × 100 = 160            */
        .encoder_ppr                  = 1000,
        .encoder_counts_per_rev       = 4000,
        .microsteps_per_count_x100    = 160,
    },
};

/* ====================================================================
   API 実装
   ==================================================================== */

const motor_profile_t *motor_spec_get_profile(motor_profile_id_t id)
{
    if (id == MOTOR_PROFILE_NONE || id >= MOTOR_PROFILE_COUNT) return NULL;
    return &s_profiles[id];
}

uint8_t motor_spec_profile_count(void)
{
    return (uint8_t)(MOTOR_PROFILE_COUNT - 1U);   /* NONE を除く */
}

uint16_t motor_spec_enc_ratio_x100(const motor_profile_t *p,
                                    uint16_t               microstep_div)
{
    if (!p || p->encoder_counts_per_rev == 0 || microstep_div == 0) return 0;
    uint32_t microsteps_per_rev = (uint32_t)p->full_steps_per_rev * microstep_div;
    return (uint16_t)(microsteps_per_rev * 100U / p->encoder_counts_per_rev);
}

uint8_t motor_spec_torque_pct(const motor_torque_point_t *curve,
                               uint8_t                     len,
                               uint32_t                    microstep_div,
                               uint32_t                    microsteps_per_sec)
{
    if (!curve || len == 0 || microstep_div == 0) return 0;

    uint32_t full_sps = microsteps_per_sec / microstep_div;

    if (full_sps <= curve[0].full_steps_per_sec) {
        return curve[0].torque_pct;
    }
    if (full_sps >= curve[len - 1U].full_steps_per_sec) {
        return curve[len - 1U].torque_pct;
    }

    for (uint8_t i = 1U; i < len; i++) {
        if (full_sps <= curve[i].full_steps_per_sec) {
            uint32_t x0  = curve[i - 1U].full_steps_per_sec;
            uint32_t x1  = curve[i].full_steps_per_sec;
            uint32_t y0  = curve[i - 1U].torque_pct;
            uint32_t y1  = curve[i].torque_pct;
            uint32_t num = y0 * (x1 - full_sps) + y1 * (full_sps - x0);
            return (uint8_t)(num / (x1 - x0));
        }
    }
    return curve[len - 1U].torque_pct;
}
