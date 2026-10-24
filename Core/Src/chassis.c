/**
 * chassis.c  两轮差速底盘：(v, w) -> 左右轮速度 -> 速度 PID(+前馈) -> PWM
 */
#include "chassis.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "pid.h"
#include "app_config.h"
#include <math.h>

static ChassisState_t s_ch;
static Pid_t          s_pid_l, s_pid_r;

static float clampf(float x, float lim)
{
    if (x > lim)  return lim;
    if (x < -lim) return -lim;
    return x;
}

static void chassis_reset_ctrl(void)
{
    Pid_Reset(&s_pid_l);
    Pid_Reset(&s_pid_r);
}

void Chassis_Init(void)
{
    s_ch.mode = CH_OFF;
    chassis_reset_ctrl();
    Motor_Init();
}

void Chassis_SetVW(float v, float w)
{
    if (s_ch.mode != CH_SPEED)
    {
        const Odom_t *o = Encoder_Get();
        chassis_reset_ctrl();
        s_ch.v_ramp = 0.5f * (o->speed_l + o->speed_r);
        s_ch.mode   = CH_SPEED;
    }
    s_ch.v_cmd = v;
    s_ch.w_cmd = w;
}

void Chassis_SetOpen(int16_t duty_l, int16_t duty_r)
{
    s_ch.mode   = CH_OPEN;
    s_ch.duty_l = duty_l;
    s_ch.duty_r = duty_r;
}

void Chassis_Stop(void)
{
    s_ch.mode = CH_OFF;
}

void Chassis_Brake(void)
{
    s_ch.mode = CH_BRAKE;
    Motor_Brake();          /* 立即生效，不等下一个控制周期 */
}

const ChassisState_t *Chassis_Get(void)
{
    return &s_ch;
}

static int16_t wheel_ctrl(Pid_t *p, float target, float meas, float dt)
{
    float out;

    p->kp      = g_param.spd_kp;
    p->ki      = g_param.spd_ki;
    p->kd      = 0.0f;
    p->i_lim   = g_param.spd_i_lim;
    p->out_lim = g_param.pwm_max;

    out = g_param.spd_kf * target + Pid_Update(p, target - meas, dt);
    return (int16_t)clampf(out, g_param.pwm_max);
}

void Chassis_Update(float dt)
{
    const Odom_t *o = Encoder_Get();

    switch (s_ch.mode)
    {
    case CH_SPEED:
    {
        float dv   = s_ch.v_cmd - s_ch.v_ramp;
        float rate = (fabsf(s_ch.v_cmd) > fabsf(s_ch.v_ramp)) ? g_param.acc_mm_s2 : g_param.dec_mm_s2;
        float half = 0.5f * g_param.wheel_track_mm * s_ch.w_cmd;

        s_ch.v_ramp += clampf(dv, rate * dt);
        s_ch.tgt_l   = s_ch.v_ramp - half;
        s_ch.tgt_r   = s_ch.v_ramp + half;
        s_ch.duty_l  = wheel_ctrl(&s_pid_l, s_ch.tgt_l, o->speed_l, dt);
        s_ch.duty_r  = wheel_ctrl(&s_pid_r, s_ch.tgt_r, o->speed_r, dt);
        Motor_Set(s_ch.duty_l, s_ch.duty_r);
        break;
    }
    case CH_OPEN:
        Motor_Set(s_ch.duty_l, s_ch.duty_r);
        break;
    case CH_BRAKE:
        Motor_Brake();
        goto idle;
    case CH_OFF:
    default:
        Motor_Coast();
    idle:
        chassis_reset_ctrl();
        s_ch.v_ramp = 0.0f;
        s_ch.tgt_l  = s_ch.tgt_r  = 0.0f;
        s_ch.duty_l = s_ch.duty_r = 0;
        break;
    }
}

float Chassis_HeadingW(float target_rad)
{
    float e = target_rad - Encoder_Get()->heading;
    return clampf(g_param.head_kp * e, g_param.w_max);
}

uint8_t Chassis_PivotW(float target_rad, float *w)
{
    float e  = target_rad - Encoder_Get()->heading;
    float ww = clampf(g_param.head_kp * e, g_param.w_pivot);

    if (fabsf(e) < DEG2RAD(g_param.pivot_tol_deg))
    {
        *w = 0.0f;
        return 1;
    }
    if (fabsf(ww) < g_param.w_pivot_min)
        ww = (e > 0.0f) ? g_param.w_pivot_min : -g_param.w_pivot_min;
    *w = ww;
    return 0;
}
