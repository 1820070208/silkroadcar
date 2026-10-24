/**
 * app.c  顶层状态机
 *   WAIT  上电等 start_delay_ms（3s），且循迹模块在线才出发
 *   RUN   循迹 + 避障；出发 finish_arm_ms（25s）后开始判终点线：
 *         循迹状态下 ≥finish_black_cnt(4) 个探头同时压黑 → 立即刹车
 *   DONE / FAULT  刹车停住
 *
 * LED(PC13)：等待慢闪 / 等待但循迹模块无数据快闪 / 运行常亮（避障接管期间 2Hz 慢闪）/ 完成熄灭 / 故障爆闪
 */
#include "app.h"
#include "app_config.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_ultrasonic.h"
#include "bsp_track.h"
#include "chassis.h"
#include "line_follow.h"
#include "obstacle.h"

AppDebug_t g_dbg;

static AppState_t s_state;
static uint32_t   s_boot_tick;
static uint32_t   s_run_tick;
static uint32_t   s_last_ctrl;
static uint8_t    s_fin_cnt;
static uint32_t   s_fin_frame;
static float      s_fin_odom;
static float      s_fin_heading;
static float      s_fin_v;
static uint8_t    s_line_status;

/* ------------------------------------------------------------------ */
static void led_set(uint8_t on)
{
    HAL_GPIO_WritePin(HW_LED_PORT, HW_LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void led_update(uint32_t now)
{
    switch (s_state)
    {
    case APP_WAIT:      led_set(Track_IsAlive(now) ? ((now / 500u) & 1u) : ((now / 100u) & 1u)); break;
    case APP_RUN:
    case APP_FINISHING:
        /* 避障接管期间（正在减速接近或绕行）改为 2Hz 慢闪，
           脱线跑车没有调试器也能用眼睛确认"避障有没有触发" */
        if (Obs_GetState() >= OBS_APPROACH)
            led_set((now / 250u) & 1u);
        else
            led_set(1);
        break;
    case APP_DONE:      led_set(0); break;
    default:            led_set((now / 50u) & 1u); break;
    }
}

static void start_run(uint32_t now)
{
    Encoder_ResetOdom();
    Track_SetRef(0.0f);
    Line_Reset();
    Obs_Reset();
    s_fin_cnt  = 0;
    s_run_tick = now;
    s_state    = APP_RUN;
}

/* 等待出发：时间到且循迹模块在线 */
static uint8_t wait_start(uint32_t now)
{
    Chassis_Stop();
    if ((uint32_t)(now - s_boot_tick) >= g_param.start_delay_ms && Track_IsAlive(now))
    {
        start_run(now);
        return 1;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* 正式比赛 / 纯循迹 */
static void mode_run(float dt, uint32_t now, uint8_t full)
{
    const Odom_t      *o = Encoder_Get();
    const TrackData_t *t = Track_Get();
    float v = 0.0f, w = 0.0f;

    switch (s_state)
    {
    case APP_WAIT:
        wait_start(now);
        break;

    case APP_RUN:
    {
        float    vc    = g_param.v_cruise;
        /* 终点武装 = 出发后计时：经过 finish_arm_ms（当前 25s）才开始判终点线。
           武装后配合"循迹状态下 ≥3 探头同时压黑"检测（斜切角度免疫），
           满足即立即刹车，无前进过冲、不限速 */
        uint8_t  armed = full && ((uint32_t)(now - s_run_tick) >= g_param.finish_arm_ms);
        ObsOut_t oo    = OBS_OUT_NONE;

        if (!Track_IsAlive(now))
        {
            Chassis_Brake();
            s_state = APP_FAULT;
            break;
        }

        /* 武装后不再降速：保持巡航速度直到压线立即刹车（v_finish 已弃用） */

        if (full)
        {
            oo = Obs_Update(dt, &v, &w);
            if (oo == OBS_OUT_TAKEOVER)
            {
                Chassis_SetVW(v, w);
                break;
            }
            if (oo == OBS_OUT_SLOW && v < vc) vc = v;
        }

        s_line_status = (uint8_t)Line_Update(dt, vc, &v, &w);
        if (s_line_status == LINE_FAIL)
        {
            Chassis_Brake();
            s_state = APP_FAULT;
            break;
        }

        /* 终点线：循迹状态下 ≥3 个探头同时见黑 → 立即刹车。
           "循迹状态"= 车正在正常沿线跑（线可见、非避障接管）：
           上面 Obs_Update 返回 TAKEOVER 时已经 break，不会走到这里，
           所以避障绕行/兜圈时扫到线不会误停；
           循迹状态下的普通线只有 1~2 个探头压黑，≥3 同时见黑只可能是
           横向的宽黑区（终点横线），进线角度不限，压上即停 */
        if (armed && t->frame_cnt != s_fin_frame)
        {
            s_fin_frame = t->frame_cnt;
            if (!t->lost && t->black_cnt >= g_param.finish_black_cnt)
            {
                Chassis_Brake();                        /* 确认终点：立即刹车 */
                s_state = APP_DONE;
            }
        }
        Chassis_SetVW(v, w);
        break;
    }

    case APP_FINISHING:                           /* 已弃用：终点确认后立即刹车，不再走此状态 */
        if (o->odom - s_fin_odom >= g_param.finish_stop_mm)
        {
            Chassis_Brake();
            s_state = APP_DONE;
        }
        else
        {
            Chassis_SetVW(s_fin_v, Chassis_HeadingW(s_fin_heading));
        }
        break;

    default:
        Chassis_Brake();
        break;
    }
}

/* 开环转电机：转 2 秒停 2 秒 */
static void mode_test_motor(uint32_t now)
{
    if ((uint32_t)(now - s_boot_tick) < g_param.start_delay_ms)
    {
        Chassis_Stop();
        return;
    }
    s_state = APP_RUN;
    if (((now - s_boot_tick) / 2000u) & 1u)
        Chassis_SetOpen(g_param.test_duty, g_param.test_duty);
    else
        Chassis_Stop();
}

/* 闭环直行 test_dist_mm 后刹车 */
static void mode_test_straight(uint32_t now)
{
    const Odom_t *o = Encoder_Get();

    if (s_state == APP_WAIT)
    {
        if ((uint32_t)(now - s_boot_tick) >= g_param.start_delay_ms) start_run(now);
        else Chassis_Stop();
        return;
    }
    if (s_state == APP_RUN)
    {
        if (o->odom >= g_param.test_dist_mm)
        {
            Chassis_Brake();
            s_state = APP_DONE;
        }
        else
        {
            Chassis_SetVW(g_param.test_speed, Chassis_HeadingW(0.0f));
        }
        return;
    }
    Chassis_Brake();
}

/* 原地左转 360 度后刹车 */
static void mode_test_pivot(uint32_t now)
{
    float w;

    if (s_state == APP_WAIT)
    {
        if ((uint32_t)(now - s_boot_tick) >= g_param.start_delay_ms) start_run(now);
        else Chassis_Stop();
        return;
    }
    if (s_state == APP_RUN)
    {
        if (Chassis_PivotW(2.0f * APP_PI, &w))
        {
            Chassis_Brake();
            s_state = APP_DONE;
        }
        else
        {
            Chassis_SetVW(0.0f, w);
        }
        return;
    }
    Chassis_Brake();
}

/* 直线避障测试：闭环直行 → 遇障碍自动绕行（不需循迹线）→ 回正继续直行
   → 总里程走满 avoid_test_mm 刹车。障碍物摆在起点 25cm 之后、终点 30cm 之前。 */
static void mode_test_avoid(float dt, uint32_t now)
{
    const Odom_t *o = Encoder_Get();
    float v = 0.0f, w = 0.0f;

    if (s_state == APP_WAIT)
    {
        if ((uint32_t)(now - s_boot_tick) >= g_param.start_delay_ms) start_run(now);
        else Chassis_Stop();
        return;
    }
    if (s_state == APP_RUN)
    {
        ObsOut_t oo = Obs_Update(dt, &v, &w);

        if (oo == OBS_OUT_TAKEOVER)     /* 避障状态机接管底盘（接近/绕行中） */
        {
            Chassis_SetVW(v, w);
            return;
        }

        if (o->odom >= g_param.avoid_test_mm)
        {
            Chassis_Brake();
            s_state = APP_DONE;
            return;
        }
        /* 无线可循：航向保持 0° 的闭环直行 */
        Chassis_SetVW(g_param.test_speed, Chassis_HeadingW(0.0f));
        return;
    }
    Chassis_Brake();
}

/* ------------------------------------------------------------------ */
static void debug_update(uint32_t now)
{
    const Odom_t         *o = Encoder_Get();
    const TrackData_t    *t = Track_Get();
    const ChassisState_t *c = Chassis_Get();

    g_dbg.app_state    = (uint8_t)s_state;
    g_dbg.line_status  = s_line_status;
    g_dbg.obs_state    = (uint8_t)Obs_GetState();
    g_dbg.obs_count    = Obs_GetCount();
    g_dbg.obs_dir      = Obs_GetDir();
    g_dbg.track_alive  = Track_IsAlive(now);
    g_dbg.track_bits   = t->bits;
    g_dbg.black_cnt    = t->black_cnt;
    g_dbg.line_pos     = t->pos_mm;
    g_dbg.track_frames = t->frame_cnt;
    g_dbg.us_f         = Us_GetMm(US_FRONT);
    g_dbg.us_l         = Us_GetMm(US_LEFT);
    g_dbg.us_r         = Us_GetMm(US_RIGHT);
    g_dbg.spd_l        = o->speed_l;
    g_dbg.spd_r        = o->speed_r;
    g_dbg.tgt_l        = c->tgt_l;
    g_dbg.tgt_r        = c->tgt_r;
    g_dbg.duty_l       = c->duty_l;
    g_dbg.duty_r       = c->duty_r;
    g_dbg.v_cmd        = c->v_cmd;
    g_dbg.w_cmd        = c->w_cmd;
    g_dbg.odom         = o->odom;
    g_dbg.heading_deg  = RAD2DEG(o->heading);
    if (s_state == APP_RUN || s_state == APP_FINISHING)
        g_dbg.run_ms = now - s_run_tick;
}

static void app_control(float dt, uint32_t now)
{
    (void)mode_run; (void)mode_test_motor; (void)mode_test_straight; (void)mode_test_pivot;
    (void)mode_test_avoid;

    Encoder_Update(dt);

#if   CFG_APP_MODE == APP_MODE_RUN
    mode_run(dt, now, 1);
#elif CFG_APP_MODE == APP_MODE_TEST_LINE
    mode_run(dt, now, 0);
#elif CFG_APP_MODE == APP_MODE_TEST_MOTOR
    mode_test_motor(now);
#elif CFG_APP_MODE == APP_MODE_TEST_STRAIGHT
    mode_test_straight(now);
#elif CFG_APP_MODE == APP_MODE_TEST_PIVOT
    mode_test_pivot(now);
#elif CFG_APP_MODE == APP_MODE_TEST_AVOID
    mode_test_avoid(dt, now);
#else /* APP_MODE_TEST_SENSOR */
    Chassis_Stop();
#endif

    Chassis_Update(dt);
    led_update(now);
    debug_update(now);
}

/* ------------------------------------------------------------------ */
void App_Init(void)
{
    Chassis_Init();     /* 内含 Motor_Init，先让电机处于停止状态 */
    Encoder_Init();
    Us_Init();
    Track_Init();
    Line_Reset();
    Obs_Reset();

    s_state     = APP_WAIT;
    s_boot_tick = HAL_GetTick();
    s_last_ctrl = s_boot_tick;
}

void App_Loop(void)
{
    uint32_t now = HAL_GetTick();
    static uint32_t s_prev_now = 0;   /* 上次控制执行的时刻，用于算真实 dt */
    float real_dt;

    Track_Process(now);
    Us_Process(now);

    if ((uint32_t)(now - s_last_ctrl) >= CFG_CTRL_PERIOD_MS)
    {
        s_last_ctrl += CFG_CTRL_PERIOD_MS;
        if ((uint32_t)(now - s_last_ctrl) > 5u * CFG_CTRL_PERIOD_MS)
            s_last_ctrl = now;   /* 被调试器暂停过，不追赶 */

        /* 真实 dt：I2C 读取会让个别周期拖长到 ~12ms，
           若仍按固定 10ms 算，编码器测速会虚高 ~20%，速度环就会一顿一顿 */
        real_dt = (float)(now - s_prev_now) * 0.001f;
        if (real_dt > 0.05f)  real_dt = 0.05f;   /* 调试器暂停等异常大间隔，截断 */
        if (real_dt < 0.002f) real_dt = 0.002f;
        s_prev_now = now;

        app_control(real_dt, now);
    }
}
