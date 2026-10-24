/**
 * line_follow.c  循迹控制
 *   正常：偏差 PD -> 角速度；偏差越大速度越低
 *   宽线：经过起点/终点线时按进入前的航向直行，不被横线带偏
 *   丢线：先保持指令一小段时间，再原地朝最后看到线的一侧转着找
 */
#include "line_follow.h"
#include "bsp_track.h"
#include "bsp_encoder.h"
#include "chassis.h"
#include "app_config.h"
#include <math.h>

static float    s_prev_e;
static float    s_de;
static uint32_t s_prev_frame;
static uint32_t s_prev_tick;
static float    s_lost_t;
static float    s_last_v, s_last_w;
static int8_t   s_last_side = 1;
static float    s_hold_heading;

static float clampf(float x, float lim)
{
    if (x > lim)  return lim;
    if (x < -lim) return -lim;
    return x;
}

void Line_Reset(void)
{
    const TrackData_t *t = Track_Get();

    s_prev_e       = t->pos_mm;
    s_de           = 0.0f;
    s_prev_frame   = t->frame_cnt;
    s_prev_tick    = HAL_GetTick();
    s_lost_t       = 0.0f;
    s_last_v       = 0.0f;
    s_last_w       = 0.0f;
    s_hold_heading = Encoder_Get()->heading;
}

void Line_SetLastSide(int8_t side)
{
    s_last_side = (side >= 0) ? 1 : -1;
}

LineStatus_t Line_Update(float dt, float v_cruise, float *v, float *w)
{
    const TrackData_t *t = Track_Get();
    const Odom_t      *o = Encoder_Get();
    LineStatus_t       st;
    float              vv, ww;

    if (!t->lost && t->black_cnt >= g_param.track_wide_cnt)
    {
        vv       = v_cruise;
        ww       = Chassis_HeadingW(s_hold_heading);
        s_lost_t = 0.0f;
        st       = LINE_WIDE;
    }
    else if (!t->lost)
    {
        float e = t->pos_mm;
        float r;

        if (t->frame_cnt != s_prev_frame)
        {
            uint32_t now = HAL_GetTick();
            float    fdt = (float)(now - s_prev_tick) * 0.001f;
            if (fdt > 0.0f && fdt < 0.1f)
                s_de = 0.6f * s_de + 0.4f * (e - s_prev_e) / fdt;
            else
                s_de = 0.0f;
            s_prev_e     = e;
            s_prev_tick  = now;
            s_prev_frame = t->frame_cnt;
        }

        ww = -(g_param.line_kp * e + g_param.line_kd * s_de);

        r = fabsf(e) / g_param.line_slow_err_mm;
        if (r > 1.0f) r = 1.0f;
        vv = v_cruise * (1.0f - g_param.line_slow_k * r);
        if (vv < g_param.v_min) vv = (v_cruise < g_param.v_min) ? v_cruise : g_param.v_min;

        if (e > 0.5f)       s_last_side = 1;
        else if (e < -0.5f) s_last_side = -1;

        s_lost_t       = 0.0f;
        s_hold_heading = o->heading;
        st             = LINE_OK;
    }
    else
    {
        s_lost_t += dt;
        if (s_lost_t < g_param.lost_keep_s)
        {
            vv = s_last_v;
            ww = s_last_w;
            st = LINE_LOST;
        }
        else if (s_lost_t < g_param.lost_keep_s + g_param.lost_spin_s)
        {
            vv = g_param.v_search;
            ww = (s_last_side > 0) ? -g_param.w_search : g_param.w_search;
            st = LINE_SEARCH;
        }
        else
        {
            vv = 0.0f;
            ww = 0.0f;
            st = LINE_FAIL;
        }
    }

    ww = clampf(ww, g_param.w_max);
    if (st == LINE_OK || st == LINE_WIDE)
    {
        s_last_v = vv;
        s_last_w = ww;
    }
    *v = vv;
    *w = ww;
    return st;
}
