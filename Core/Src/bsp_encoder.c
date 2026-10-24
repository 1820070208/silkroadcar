/**
 * bsp_encoder.c  霍尔编码器测速 + 里程计
 * 左轮：TIM4 编码器模式(PB6/PB7)  右轮：TIM1 编码器模式(PA8/PA9)
 */
#include "bsp_encoder.h"
#include "app_config.h"
#include <string.h>

#define SPEED_LPF_ALPHA  0.5f

static Odom_t   s_odom;
static uint16_t s_last_l, s_last_r;

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&HW_ENC_L_TIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&HW_ENC_R_TIM, TIM_CHANNEL_ALL);
    s_last_l = (uint16_t)__HAL_TIM_GET_COUNTER(&HW_ENC_L_TIM);
    s_last_r = (uint16_t)__HAL_TIM_GET_COUNTER(&HW_ENC_R_TIM);
    memset(&s_odom, 0, sizeof(s_odom));
}

void Encoder_Update(float dt)
{
    uint16_t cl = (uint16_t)__HAL_TIM_GET_COUNTER(&HW_ENC_L_TIM);
    uint16_t cr = (uint16_t)__HAL_TIM_GET_COUNTER(&HW_ENC_R_TIM);
    int16_t  dl = (int16_t)(uint16_t)(cl - s_last_l);
    int16_t  dr = (int16_t)(uint16_t)(cr - s_last_r);
    float    k  = APP_PI * g_param.wheel_diam_mm / g_param.counts_per_rev;
    float    sl, sr;

    s_last_l = cl;
    s_last_r = cr;
#if CFG_ENC_L_REVERSE
    dl = (int16_t)-dl;
#endif
#if CFG_ENC_R_REVERSE
    dr = (int16_t)-dr;
#endif

    sl = (float)dl * k;
    sr = (float)dr * k;

    s_odom.cnt_l   += dl;
    s_odom.cnt_r   += dr;
    s_odom.dist_l  += sl;
    s_odom.dist_r  += sr;
    s_odom.odom    += 0.5f * (sl + sr);
    s_odom.heading += (sr - sl) / g_param.wheel_track_mm;

    if (dt > 0.0f)
    {
        s_odom.speed_l += SPEED_LPF_ALPHA * (sl / dt - s_odom.speed_l);
        s_odom.speed_r += SPEED_LPF_ALPHA * (sr / dt - s_odom.speed_r);
    }
}

void Encoder_ResetOdom(void)
{
    s_odom.cnt_l   = 0;
    s_odom.cnt_r   = 0;
    s_odom.dist_l  = 0.0f;
    s_odom.dist_r  = 0.0f;
    s_odom.odom    = 0.0f;
    s_odom.heading = 0.0f;
}

const Odom_t *Encoder_Get(void)
{
    return &s_odom;
}
