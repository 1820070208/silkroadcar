/**
 * app_config.c
 * 运行期可调参数默认值。调参时可在 Live Expressions 里直接改 g_param.xxx，
 * 调好后把数值抄回这里。
 */
#include "app_config.h"

AppParam_t g_param =
{
    /* ---------- 机械 ---------- */
    .wheel_diam_mm       = 48.0f,
    .wheel_track_mm      = 130.0f,
    .counts_per_rev      = 13.0f * 4.0f * 20.409f,  /* 13 线 × 4 倍频 × 1:20.409 ≈ 1061 */

    /* ---------- 速度环 ---------- */
    .pwm_max             = 900.0f,
    .spd_kf              = 0.50f,
    .spd_kp              = 1.0f,
    .spd_ki              = 6.0f,
    .spd_i_lim           = 300.0f,
    .acc_mm_s2           = 1500.0f,
    .dec_mm_s2           = 3000.0f,

    /* ---------- 循迹 ---------- */
    .track_pitch_mm      = 12.5f,
    .track_wide_cnt      = 5,
    .v_cruise            = 300.0f,   /* 稳定版：全程统一 370 */
    .v_min               = 180.0f,
    .line_kp             = 0.090f,   /* 配平 370：0.085 → 0.090 */
    .line_kd             = 0.0055f,  /* 同步升阻尼 */
    .line_slow_k         = 0.4f,
    .line_slow_err_mm    = 40.0f,
    .w_max               = 6.0f,   /* 370 速度下防内轮倒转：5×65=325 < 370（6 会倒转）*/
    .lost_keep_s         = 0.15f,
    .lost_spin_s         = 3.0f,
    .w_search            = 2.5f,
    .v_search            = 60.0f,

    /* ---------- 航向保持 / 原地转 ---------- */
    .head_kp             = 5.0f,
    .w_pivot             = 3.5f,
    .w_pivot_min         = 0.8f,
    .pivot_tol_deg       = 3.0f,

    /* ---------- 启动 / 终点 ---------- */
    .start_delay_ms      = 3000,
    .finish_arm_ms       = 26000,   /* 出发 24s 后开始判终点线；⚠️ 若单圈用时<25s 会跑第二圈才停 */
    .finish_arm_mm       = 5000.0f,   /* （已弃用） */
    .v_finish            = 200.0f,    /* （已弃用：武装后不再降速，保持巡航直到压线急停） */
    .finish_black_cnt    = 4,   /* 循迹状态下 ≥4 探头同时见黑=终点横线，立即停车 */
    .finish_black_mm     = 20.0f,   /* （已弃用：终点确认不再需要里程累计） */
    .finish_confirm      = 2,   /* （已弃用） */
    .finish_stop_mm      = 30.0f,   /* （已弃用：确认后立即刹车，不再前进过冲段） */

    /* ---------- 避障（稳定版：识别早、减速段长、圆弧大、全员同参数）---------- */
    .obs_enable          = 1,
    .obs_max_count       = 0,
    .obs_arm_mm          = 200.0f,
    .obs_rearm_mm        = 300.0f,
    .obs_detect_mm       = 350,   /* 识别距离缩短 10cm：450 → 350 */
    .obs_decide_mm       = 250,   /* 同步 -100：减速段仍保持 100mm（350-250） */
    .obs_diag_trigger_mm = 280,   /* 斜前早触发（配 2 次防抖） */
    .obs_side_seen_mm    = 400,
    .obs_side_margin_mm  = 50,
    .obs_default_dir     = +1,
    .v_approach          = 250.0f,
    .v_avoid             = 250.0f,
    .v_avoid_blind       = 220.0f,
    .obs_turn_out_deg    = 75.0f,
    .obs_shift_mm        = 150.0f,
    .obs_pass_min_mm     = 200.0f,
    .obs_pass_max_mm     = 800.0f,
    .obs_clear_mm        = 250,
    .obs_clear_extra_mm  = 150.0f,
    .obs_return_deg      = 75.0f,
    .obs_return_max_mm   = 400.0f,
    .obs_rejoin_fwd_mm   = 80.0f,
    .obs_verify_mm       = 20.0f,
    .w_arc               = 1.8f,   /* 避障1专属：荡出 R = 250/1.8 ≈ 139mm（小圆） */
    .w_arc_back          = 1.2f,   /* 避障1专属：回程 R = 250/1.2 ≈ 208mm；2~4 次在 finish_avoid 恢复 */
    .arc_out_deg         = 55.0f,  /* 荡出角 -10°：让开量 ≈ 139×(1-cos55°) ≈ 80mm（不足由追加转向补，最多+45°） */
    .arc_rejoin_deg      = 45.0f,
    .obs_state_timeout_s = 4.0f,
    .v_rejoin            = 250.0f,
    .rejoin_mm           = 250.0f,   /* 回线后慢行区加长：稳住再提速 */

    /* ---------- 超声波 ---------- */
    .us_slot_ms          = 15,
    .us_max_mm           = 2000,

    /* ---------- 测试模式 ---------- */
    .test_duty           = 750,
    .test_speed          = 300.0f,
    .test_dist_mm        = 1000.0f,
    .avoid_test_mm       = 2000.0f,
};
