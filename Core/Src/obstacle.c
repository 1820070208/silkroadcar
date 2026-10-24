/**
 * obstacle.c  避障状态机（当前策略：圆弧兜圈）
 *
 * 触发后不再分"转出/斜行/转平/平行/转回"多段停车动作，而是两个连续圆弧：
 *
 *   ARC_OUT ：恒速 v_avoid + 恒角速度 w_arc 向障碍外侧荡出，
 *             航向荡到 h0 - dir*arc_out_deg 为止；
 *   ARC_BACK：曲率反向，划大弧线绕过障碍并朝线荡回，
 *             途中任意探头看到线 → 20mm 验证窗（防赛道黑字）→ 真线立即切循迹；
 *             兜过头（航向越过 h0 + dir*arc_out_deg 仍没见线）→ 转 RETURN 斜行兜底。
 *
 *   圆弧半径 R = v_avoid / w_arc：调 w_arc 即调圈子大小。
 *
 * 旧的"三段梯形"状态（TURN_OUT/SHIFT/TURN_PARALLEL/TURN_BACK）保留在代码里但
 * 正常流程不再经过（decide 直接进 ARC_OUT）；RETURN/REJOIN 仍作为兜底路径。
 *
 * 判断障碍在哪一侧：接近阶段记录左/右斜前超声波的最小读数，读数小的一侧就是障碍侧；
 * 分不清时按 obs_default_dir 绕。所有角度用编码器航向，距离用编码器里程。
 */
#include "obstacle.h"
#include "bsp_ultrasonic.h"
#include "bsp_encoder.h"
#include "bsp_track.h"
#include "chassis.h"
#include "line_follow.h"
#include "app_config.h"
#include <math.h>

#define OBS_HIT_N  2       /* 触发避障所需：正前方连续低于阈值的测量次数 */
#define OBS_SIDE_HIT_N  2  /* 斜前触发所需：同一侧连续低于阈值的测量次数（稳定版恢复防抖）*/
#define OBS_EXTRA_STEP  15.0f  /* 荡出后正前仍有障碍时：每次追加的转角（度）*/
#define OBS_EXTRA_MAX   45.0f  /* 追加转角的上限（防止无限转圈）*/

static ObsState_t s_st;
static int8_t     s_dir;
static float      s_h0;             /* 开始绕行时的航向 = 线的方向 */
static float      s_seg_start;      /* 当前步骤起点里程 */
static float      s_state_t;
static uint16_t   s_min_l, s_min_r;
static uint8_t    s_far;
static uint8_t    s_hit;            /* 正前方连续低于阈值的次数（达到 OBS_HIT_N 才触发） */
static uint32_t   s_front_seq;
static uint16_t   s_l_seq, s_r_seq; /* 左右斜前各自的测量序号（识别"新测量"用） */
static uint8_t    s_l_hit, s_r_hit; /* 左右斜前连续低于阈值的次数 */
static float      s_rearm_until;
static float      s_rejoin_until;
static float      s_last_near;
static uint8_t    s_count;
static uint16_t   s_seen_cnt;       /* 回线验证窗内"看到线"的控制周期数 */
static uint16_t   s_seen_total;     /* 回线验证窗内总控制周期数 */
static uint8_t    s_rejoin_from_pass; /* 验证窗的来源：1=从平行通过进入 0=从斜行找线进入 */
static float      s_rejoin_hold;    /* 验证窗期间保持的航向 */
static uint8_t    s_rejoin_back;    /* 验证失败后退回的目标：0=RETURN 1=PASS 2=ARC_BACK */
static uint8_t    s_blind;          /* 盲绕标志：decide() 分不清障碍侧 = 1（弧线降速） */
static float      s_extra_deg;      /* 荡出段追加转角（正前仍有障碍时逐次累加） */

static void go(ObsState_t st)
{
    s_st        = st;
    s_state_t   = 0.0f;
    s_seg_start = Encoder_Get()->odom;
    if (st == OBS_REJOIN_FWD)
    {
        s_seen_cnt   = 0;   /* 回线验证窗：重新统计"看到线"的占比 */
        s_seen_total = 0;
    }
}

static void finish_avoid(void)
{
    float odom = Encoder_Get()->odom;

    s_count++;
    s_hit = 0;
    s_st  = OBS_IDLE;

    /* 稳定版策略：慢速、识别早、减速段长。
       参数结构 = app_config.c 默认值（避障 1 专属：小圆）+ count 块按次恢复：
       避障 1 用默认小圆（139/208mm），避障 2~4 恢复标准圆（167/250mm） */

    if (s_count == 1u)
    {
        /* 第 1 次避障完成后 = 第 2 次的参数：恢复标准圆 + 保留第 2 次专属特例。
           （速度/kp/kd 已全程统一 370，无需在此切换） */
        g_param.w_arc       = 1.5f;    /* 荡出圆恢复：R ≈ 250/1.5 ≈ 167mm */
        g_param.w_arc_back  = 0.85f;   /* 第 2 次专属大回程圆：R ≈ 250/0.85 ≈ 294mm */
        g_param.arc_out_deg = 50.0f;   /* 第 2 次专属荡出角（单独调小） */
    }
    if (s_count == 2u || s_count == 3u)
    {
        /* 第 2、3 次避障完成后 = 第 3、4 次的参数：恢复标准回程圆 */
        g_param.w_arc       = 1.5f;    /* R ≈ 167mm */
        g_param.w_arc_back  = 1.0f;    /* R ≈ 250mm */
        g_param.v_cruise    = 370.0f;
    }

    /* 解除武装/回线限速窗口使用全局默认值（obs_rearm_mm = 300） */
    s_rearm_until  = odom + g_param.obs_rearm_mm;
    s_rejoin_until = odom + g_param.rejoin_mm;

    Track_SetRef(Track_Get()->pos_mm);
    Line_Reset();
    Line_SetLastSide(s_dir);   /* 从左绕回来线在右边，反之在左边 */
}

static void decide(void)
{
    uint16_t m = g_param.obs_side_margin_mm;
    uint8_t  side_ok = 0;

    if (s_min_l < g_param.obs_side_seen_mm && (uint32_t)s_min_l + m < s_min_r)
    {
        s_dir = -1;                         /* 障碍偏左，从右边绕 */
        side_ok = 1;
    }
    else if (s_min_r < g_param.obs_side_seen_mm && (uint32_t)s_min_r + m < s_min_l)
    {
        s_dir = +1;                         /* 障碍偏右，从左边绕 */
        side_ok = 1;
    }
    else
        s_dir = (g_param.obs_default_dir >= 0) ? 1 : -1;

    /* 盲绕保护：分不清障碍在哪侧（默认方向兜底）时，弧线降速行驶，
       给赌错方向留出反应余地；方向判定成功则按正常速度绕行 */
    s_blind = (uint8_t)!side_ok;
    s_extra_deg = 0.0f;   /* 每次避障的追加转角从 0 重新计 */

    s_h0 = Encoder_Get()->heading;
    go(OBS_ARC_OUT);      /* 新策略：圆弧兜圈（连续无停车） */
}

void Obs_Reset(void)
{
    s_st           = OBS_IDLE;
    s_dir          = 1;
    s_hit          = 0;
    s_far          = 0;
    s_count        = 0;
    s_l_hit        = 0;
    s_r_hit        = 0;
    s_rearm_until  = 0.0f;
    s_rejoin_until = 0.0f;
    s_front_seq    = Us_GetSeq(US_FRONT);
    s_l_seq        = (uint16_t)Us_GetSeq(US_LEFT);
    s_r_seq        = (uint16_t)Us_GetSeq(US_RIGHT);
}

ObsState_t Obs_GetState(void) { return s_st; }
uint8_t    Obs_GetCount(void) { return s_count; }
int8_t     Obs_GetDir(void)   { return s_dir; }

ObsOut_t Obs_Update(float dt, float *v, float *w)
{
    const Odom_t *o     = Encoder_Get();
    uint16_t      df    = Us_GetMm(US_FRONT);
    uint16_t      dl    = Us_GetMm(US_LEFT);
    uint16_t      dr    = Us_GetMm(US_RIGHT);
    uint8_t       fresh = 0;
    float         trav  = o->odom - s_seg_start;
    float         th_out = DEG2RAD(g_param.obs_turn_out_deg);
    float         th_ret = DEG2RAD(g_param.obs_return_deg);

    if (Us_GetSeq(US_FRONT) != s_front_seq)
    {
        s_front_seq = Us_GetSeq(US_FRONT);
        fresh = 1;
    }

    s_state_t += dt;
    {
        /* 圆弧兜圈允许转更久（大圆一圈就要 6s+），超时上限放大 3 倍；
           超时仍然兜底：交还循迹，让循迹的找线逻辑接手 */
        float t_lim = (s_st == OBS_ARC_OUT || s_st == OBS_ARC_BACK)
                      ? g_param.obs_state_timeout_s * 3.0f
                      : g_param.obs_state_timeout_s;

        if (s_st >= OBS_TURN_OUT && s_state_t > t_lim)
        {
            finish_avoid();
            return OBS_OUT_NONE;
        }
    }

    switch (s_st)
    {
    case OBS_IDLE:
    {
        uint8_t armed = g_param.obs_enable &&
                        o->odom >= g_param.obs_arm_mm &&
                        o->odom >= s_rearm_until &&
                        (g_param.obs_max_count == 0u || s_count < g_param.obs_max_count);
        uint16_t dt_mm = g_param.obs_diag_trigger_mm;

        /* 斜前触发：左右超声波各自连续 OBS_SIDE_HIT_N 次新测量低于阈值才触发，
           （正前探头视野窄，斜着伸过来的障碍它可能一直看不到，靠斜前补位）
           防抖与正前一致，滤掉瞬时误反射 */
        if (armed && dt_mm > 0u)
        {
            uint8_t l_new = (Us_GetSeq(US_LEFT)  != s_l_seq);
            uint8_t r_new = (Us_GetSeq(US_RIGHT) != s_r_seq);

            if (l_new) s_l_seq = Us_GetSeq(US_LEFT);
            if (r_new) s_r_seq = Us_GetSeq(US_RIGHT);

            if (l_new && dl < dt_mm)
            {
                if (++s_l_hit >= OBS_SIDE_HIT_N)
                {
                    s_min_l = dl;
                    s_min_r = dr;
                    decide();
                    break;
                }
            }
            else if (l_new)
            {
                s_l_hit = 0;
            }

            if (r_new && dr < dt_mm)
            {
                if (++s_r_hit >= OBS_SIDE_HIT_N)
                {
                    s_min_l = dl;
                    s_min_r = dr;
                    decide();
                    break;
                }
            }
            else if (r_new)
            {
                s_r_hit = 0;
            }
        }
        /* 触发确认：连续 OBS_HIT_N 次低于阈值 → OBS_APPROACH 减速接近 → decide()。
           四次避障统一走"确认→减速→转向"流程（第 3 次的无缓冲直通已取消） */
        if (armed && fresh)
        {
            if (df < g_param.obs_detect_mm)
            {
                if (++s_hit >= OBS_HIT_N)
                {
                    s_min_l = dl;
                    s_min_r = dr;
                    s_far   = 0;
                    go(OBS_APPROACH);
                }
            }
            else
            {
                s_hit = 0;
            }
        }
        if (o->odom < s_rejoin_until)
        {
            *v = g_param.v_rejoin;
            return OBS_OUT_SLOW;
        }
        return OBS_OUT_NONE;
    }

    case OBS_APPROACH:
        if (dl < s_min_l) s_min_l = dl;
        if (dr < s_min_r) s_min_r = dr;
        if (fresh)
        {
            if (df > g_param.obs_detect_mm + 100u)
            {
                if (++s_far >= 3u)   /* 误报：障碍消失 */
                {
                    s_st   = OBS_IDLE;
                    s_hit  = 0;
                    return OBS_OUT_NONE;
                }
            }
            else
            {
                s_far = 0;
            }
        }
        if (df <= g_param.obs_decide_mm)
        {
            decide();
            break;
        }
        *v = g_param.v_approach;
        return OBS_OUT_SLOW;

    default:
        break;
    }

    /* ---------------- 以下为接管底盘的绕行步骤 ---------------- */
    switch (s_st)
    {
    case OBS_TURN_OUT:
        *v = 0.0f;
        if (Chassis_PivotW(s_h0 - s_dir * th_out, w)) go(OBS_SHIFT);   /* 符号已按实测翻转 */
        break;

    case OBS_SHIFT:
        *v = g_param.v_avoid;
        *w = Chassis_HeadingW(s_h0 - s_dir * th_out);   /* 与 TURN_OUT 同步翻转 */
        if (trav >= g_param.obs_shift_mm / sinf(th_out)) go(OBS_TURN_PARALLEL);
        break;

    case OBS_TURN_PARALLEL:
        *v = 0.0f;
        if (Chassis_PivotW(s_h0, w))
        {
            go(OBS_PASS);
            s_last_near = o->odom;
        }
        break;

    case OBS_PASS:
    {
        uint16_t side = (s_dir > 0) ? dr : dl;   /* 障碍所在一侧的斜前超声波 */
        *v = g_param.v_avoid;
        *w = Chassis_HeadingW(s_h0);
        if (side < g_param.obs_clear_mm) s_last_near = o->odom;
        /* 平行通过期间也盯着循迹线：赛道弯道把线送到探头下面时提前接上。
           同样走 20mm 验证窗防黑字误判，验证失败就回来继续平行通过 */
        if (!Track_Get()->lost)
        {
            s_rejoin_from_pass = 1;
            s_rejoin_hold      = s_h0;
            s_rejoin_back      = 1;
            go(OBS_REJOIN_FWD);
            break;
        }
        if ((trav >= g_param.obs_pass_min_mm && o->odom - s_last_near >= g_param.obs_clear_extra_mm) ||
            trav >= g_param.obs_pass_max_mm)
            go(OBS_TURN_BACK);
        break;
    }

    case OBS_TURN_BACK:
        *v = 0.0f;
        if (Chassis_PivotW(s_h0 + s_dir * th_ret, w)) go(OBS_RETURN);   /* 符号已按实测翻转 */
        break;

    case OBS_RETURN:
        *v = g_param.v_avoid;
        *w = Chassis_HeadingW(s_h0 + s_dir * th_ret);   /* 与 TURN_BACK 同步翻转 */
        if (!Track_Get()->lost && trav > 20.0f)
        {
            s_rejoin_from_pass = 0;
            s_rejoin_hold      = s_h0 + s_dir * th_ret;
            s_rejoin_back      = 0;
            go(OBS_REJOIN_FWD);
        }
        else if (trav >= g_param.obs_shift_mm / sinf(th_ret))
        {
            /* 按几何推算已横向回到原车道（直线避障测试没有线可找，靠这条退出） */
            s_rejoin_from_pass = 0;
            s_rejoin_hold      = s_h0 + s_dir * th_ret;
            s_rejoin_back      = 0;
            go(OBS_REJOIN_FWD);
        }
        else if (trav >= g_param.obs_return_max_mm)
        {
            finish_avoid();   /* 没找到线：交给循迹的找线逻辑 */
            return OBS_OUT_NONE;
        }
        break;

    case OBS_REJOIN_FWD:
        /* 回线验证窗：看到线后继续前进 obs_verify_mm，期间统计"看到线"的占比——
           真线会一直跟着探头，赛道上印的黑字只有两三个探头宽，走 20mm 就甩掉了。
           窗口结束时线可见占比过半 → 真线，立刻交还循迹；
           占比不足 → 误判（黑字等），退回进入前的状态继续原动作。
           期间保持进入时刻记录的 s_rejoin_hold 航向（各来源不同）。
           "看到线"的定义 = ≥2 个探头压黑：只擦到线边缘（1 个探头）不算——
           浅荡出后车身还斜着，单探头擦线就切循迹会立刻丢线、退化成找线旋转 */
        {
            const TrackData_t *t = Track_Get();
            *v = g_param.v_avoid;
            *w = Chassis_HeadingW(s_rejoin_hold);
            s_seen_total++;
            if (t->black_cnt >= 2u) s_seen_cnt++;
            if (trav >= g_param.obs_verify_mm)
            {
                if (s_seen_cnt * 2u >= s_seen_total)
                {
                    finish_avoid();               /* 验证通过：立即切循迹 */
                    *v = g_param.v_rejoin;
                    return OBS_OUT_SLOW;
                }
                if      (s_rejoin_back == 1) go(OBS_PASS);       /* 验证失败：是黑字 */
                else if (s_rejoin_back == 2) go(OBS_ARC_BACK);
                else                         go(OBS_RETURN);
            }
        }
        break;

    case OBS_ARC_OUT:
        /* 新策略·第 1 段：恒速 + 恒角速度向外荡出（与原 TURN_OUT 同方向），
           航向荡到 arc_out_deg(+追加角) 即转第 2 段。全程不停车。
           盲绕（decide 分不清障碍侧）时弧线降速。
           追加机制（稳定版：四次全部生效）：荡到位后再探一次正前——仍有障碍就
           多转 OBS_EXTRA_STEP 度再看一眼（最多 OBS_EXTRA_MAX 度），干净了才回程 */
        *v = s_blind ? g_param.v_avoid_blind : g_param.v_avoid;
        *w = -s_dir * g_param.w_arc;
        {
            float target = s_h0 - s_dir * DEG2RAD(g_param.arc_out_deg + s_extra_deg);
            if ((target - o->heading) * (-s_dir) <= 0.03f)
            {
                if (df < g_param.obs_detect_mm && s_extra_deg < OBS_EXTRA_MAX)
                    s_extra_deg += OBS_EXTRA_STEP;   /* 正前仍有障碍：多转一点再看 */
                else
                    go(OBS_ARC_BACK);
            }
        }
        break;

    case OBS_ARC_BACK:
        /* 新策略·第 2 段：曲率反向往回兜，半径比荡出段更大（w_arc_back < w_arc），
           弧线更平缓：前进更多、横收回得更晚更缓，不容易冲过头回不来。
           认线条件：看到线（任意角度）→ 20mm 验证窗（防赛道黑字）→ 通过即切循迹。
           注意：不再要求航向接近原方向——大圆兜圈的几何决定了每次与线相交时
           车身都比较斜（约 90°），若设航向门槛会永远凑不上、一圈圈空转。
           切回循迹时航向偏差由循迹 PD 修正（w_max 兜得住）。
           兜圈兜到 12s 超时（超时上限=3×obs_state_timeout_s）仍没线 → 交给找线逻辑 */
        *v = s_blind ? g_param.v_avoid_blind : g_param.v_avoid;   /* 盲绕降速同样作用于回程弧 */
        *w = +s_dir * g_param.w_arc_back;   /* 回程用更小角速度 = 更大半径 = 更平缓 */
        if (!Track_Get()->lost)
        {
            s_rejoin_from_pass = 0;
            s_rejoin_hold      = o->heading;   /* 就地取当前航向，保持弧线末端方向 */
            s_rejoin_back      = 2;
            go(OBS_REJOIN_FWD);
        }
        break;

    case OBS_REJOIN_ALIGN:                    /* 保留：验证通过后不再走此步（免对正直接切循迹） */
        *v = 0.0f;
        if (Chassis_PivotW(s_h0, w))
        {
            finish_avoid();
            *v = g_param.v_rejoin;
            return OBS_OUT_SLOW;
        }
        break;

    default:
        return OBS_OUT_NONE;
    }
    return OBS_OUT_TAKEOVER;
}
