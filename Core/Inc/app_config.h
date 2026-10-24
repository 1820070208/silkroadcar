/**
 * app_config.h
 * 全局配置：
 *   1. 编译期硬件配置（引脚、方向翻转、运行模式）—— 改完需要重新编译
 *   2. 运行期可调参数 g_param —— 可在 STM32CubeIDE 调试时用 Live Expressions 在线修改
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"
#include <stdint.h>

/* ======================================================================
 *                         一、运行模式（编译期）
 * ====================================================================== */
#define APP_MODE_RUN            0  /* 正式比赛：循迹 + 避障 + 终点停车                 */
#define APP_MODE_TEST_MOTOR     1  /* 开环转电机：检查电机转向、编码器方向              */
#define APP_MODE_TEST_SENSOR    2  /* 电机不动：只看循迹、超声波数据                    */
#define APP_MODE_TEST_STRAIGHT  3  /* 闭环直行 test_dist_mm 后刹车：调速度环、标定轮径  */
#define APP_MODE_TEST_PIVOT     4  /* 原地左转 360 度后刹车：标定轮距                   */
#define APP_MODE_TEST_LINE      5  /* 只循迹：不避障、不判终点                          */
#define APP_MODE_TEST_AVOID     6  /* 直行避障测试：直行→遇障碍绕行→回正→走满距离刹车   */

/* 首次烧录默认是传感器测试模式（车不会动），全部测试通过后改成 APP_MODE_RUN */
#ifndef CFG_APP_MODE
#define CFG_APP_MODE            APP_MODE_RUN     //换挡！//
#endif

/* 控制周期 */
#define CFG_CTRL_PERIOD_MS      10

/* ======================================================================
 *                         二、方向修正（编译期）
 *  用 APP_MODE_TEST_MOTOR 测：两轮都应向前转，且 g_dbg.spd_l / spd_r 都为正
 * ====================================================================== */
#define CFG_MOTOR_L_REVERSE     0  /* 左轮给正占空比时往后转，改成 1 */
#define CFG_MOTOR_R_REVERSE     1  /* 右轮给正占空比时往后转，改成 1 */
#define CFG_ENC_L_REVERSE       0  /* 左轮向前转时 spd_l 为负，改成 1 */
#define CFG_ENC_R_REVERSE       1  /* 右轮向前转时 spd_r 为负，改成 1 */

/* ======================================================================
 *                     三、8 路循迹模块（亚博 YB-MUX04）
 * ====================================================================== */
#define CFG_TRACK_BLACK_LEVEL   0    /* 探头压在黑线上时 xN 的值（0 或 1，实测确认）      */
#define CFG_TRACK_X1_IS_LEFT    0    /* 从车上方朝车头方向看，X1 在最左边填 1，最右边填 0 */
#define CFG_TRACK_BOOT_MS       1000 /* 上电后等模块启动多久再发配置指令                 */
#define CFG_TRACK_TIMEOUT_MS    200  /* 超过这么久收不到数据，判定模块掉线               */

/* ======================================================================
 *                         四、硬件资源（对应已打样 PCB）
 * ====================================================================== */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

#define HW_PWM_TIM              htim2          /* PA0 = PWMA(左)  PA1 = PWMB(右) */
#define HW_PWM_CH_L             TIM_CHANNEL_1
#define HW_PWM_CH_R             TIM_CHANNEL_2
#define HW_ENC_L_TIM            htim4          /* PB6 / PB7 */
#define HW_ENC_R_TIM            htim1          /* PA8 / PA9 */
#define HW_US_TIM               htim3          /* 1MHz 自由计数，超声波计时用 */
#define HW_TRACK_UART           huart2         /* PA2 / PA3 */
#define HW_TRACK_UART_INSTANCE  USART2

#define HW_AIN1_PORT            GPIOB
#define HW_AIN1_PIN             GPIO_PIN_12
#define HW_AIN2_PORT            GPIOB
#define HW_AIN2_PIN             GPIO_PIN_13
#define HW_BIN1_PORT            GPIOB
#define HW_BIN1_PIN             GPIO_PIN_14
#define HW_BIN2_PORT            GPIOB
#define HW_BIN2_PIN             GPIO_PIN_15

#define HW_US_F_TRIG_PORT       GPIOA
#define HW_US_F_TRIG_PIN        GPIO_PIN_6
#define HW_US_F_ECHO_PORT       GPIOA
#define HW_US_F_ECHO_PIN        GPIO_PIN_7
#define HW_US_L_TRIG_PORT       GPIOB
#define HW_US_L_TRIG_PIN        GPIO_PIN_0
#define HW_US_L_ECHO_PORT       GPIOB
#define HW_US_L_ECHO_PIN        GPIO_PIN_1
#define HW_US_R_TRIG_PORT       GPIOB
#define HW_US_R_TRIG_PIN        GPIO_PIN_8
#define HW_US_R_ECHO_PORT       GPIOB
#define HW_US_R_ECHO_PIN        GPIO_PIN_9

#define HW_LED_PORT             GPIOC          /* 核心板 PC13 LED，低电平点亮 */
#define HW_LED_PIN              GPIO_PIN_13

#define APP_PI                  3.14159265f
#define DEG2RAD(d)              ((d) * (APP_PI / 180.0f))
#define RAD2DEG(r)              ((r) * (180.0f / APP_PI))

/* ======================================================================
 *                   五、运行期可调参数（默认值见 app_config.c）
 *  速度单位 mm/s，角速度 rad/s（左转为正），距离 mm，占空比为千分比
 * ====================================================================== */
typedef struct
{
    /* ---------- 机械 ---------- */
    float    wheel_diam_mm;       /* 轮径                                   */
    float    wheel_track_mm;      /* 等效轮距（用原地转 360 度标定）         */
    float    counts_per_rev;      /* 轮子转一圈的编码器计数（4 倍频后）      */

    /* ---------- 速度环 ---------- */
    float    pwm_max;             /* 占空比上限（千分比）                    */
    float    spd_kf;              /* 前馈：每 1mm/s 目标速度给多少占空比     */
    float    spd_kp;
    float    spd_ki;
    float    spd_i_lim;           /* 积分限幅（千分比）                      */
    float    acc_mm_s2;           /* 加速度限制                              */
    float    dec_mm_s2;           /* 减速度限制                              */

    /* ---------- 循迹 ---------- */
    float    track_pitch_mm;      /* 相邻探头间距                            */
    uint8_t  track_wide_cnt;      /* 黑点数 >= 该值视为宽线（起点/终点线）   */
    float    v_cruise;            /* 巡航速度                                */
    float    v_min;               /* 大偏差时的最低速度                      */
    float    line_kp;             /* 偏差(mm) -> 角速度(rad/s)               */
    float    line_kd;
    float    line_slow_k;         /* 偏差越大越减速：0 不减速，1 减到 0      */
    float    line_slow_err_mm;    /* 偏差达到该值时减速最多                  */
    float    w_max;               /* 角速度上限                              */
    float    lost_keep_s;         /* 丢线后先保持原指令多久                  */
    float    lost_spin_s;         /* 然后原地朝丢线侧转着找线多久            */
    float    w_search;            /* 找线角速度                              */
    float    v_search;            /* 找线时的前进速度（0 为原地转）          */

    /* ---------- 航向保持 / 原地转 ---------- */
    float    head_kp;             /* 航向误差(rad) -> 角速度(rad/s)          */
    float    w_pivot;             /* 原地转最大角速度                        */
    float    w_pivot_min;         /* 原地转最小角速度（克服静摩擦）          */
    float    pivot_tol_deg;       /* 原地转到位误差                          */

    /* ---------- 启动 / 终点 ---------- */
    uint32_t start_delay_ms;      /* 上电后多久出发                          */
    uint32_t finish_arm_ms;       /* 出发后经过此时长(ms)才开始判终点线      */
    float    finish_arm_mm;       /* （已弃用：终点武装改为按时间）          */
    float    v_finish;            /* 终点判定开启后的限速                    */
    uint8_t  finish_black_cnt;    /* 黑点数 >= 该值视为"压在宽线上"          */
    float    finish_black_mm;     /* 压线状态累计行驶该距离才确认终点（斜切免疫）*/
    uint8_t  finish_confirm;      /* （已弃用：终点确认改为按里程累计）      */
    float    finish_stop_mm;      /* 检测到终点线后再走多远刹车              */

    /* ---------- 避障 ---------- */
    uint8_t  obs_enable;
    uint8_t  obs_max_count;       /* 最多避几次障，0 为不限                  */
    float    obs_arm_mm;          /* 出发后多远才开启避障（躲开起点光电门）  */
    float    obs_rearm_mm;        /* 一次避障结束后多远内不再触发            */
    uint16_t obs_detect_mm;       /* 前超声波小于该值：进入接近阶段          */
    uint16_t obs_decide_mm;       /* 前超声波小于该值：判断方向并开始绕行    */
    uint16_t obs_diag_trigger_mm; /* 斜前超声波小于该值也直接触发，0 为关闭  */
    uint16_t obs_side_seen_mm;    /* 斜前读数小于该值才算“看到了障碍物”     */
    uint16_t obs_side_margin_mm;  /* 左右读数相差超过该值才认为分得清左右    */
    int8_t   obs_default_dir;     /* 分不清时默认从哪边绕：+1 左，-1 右      */
    float    v_approach;          /* 接近障碍物时的速度                      */
    float    v_avoid;             /* 绕行时的速度                            */
    float    v_avoid_blind;       /* 盲绕速度：decide 分不清障碍侧时的降速绕行 */
    float    obs_turn_out_deg;    /* 第 1 步：原地转出的角度                 */
    float    obs_shift_mm;        /* 第 2 步：横向让出的距离                 */
    float    obs_pass_min_mm;     /* 第 4 步：平行通过的最短距离             */
    float    obs_pass_max_mm;     /* 第 4 步：平行通过的最长距离             */
    uint16_t obs_clear_mm;        /* 障碍侧斜前读数大于该值视为已离开障碍物  */
    float    obs_clear_extra_mm;  /* 离开后再多走多远（约一个车长）          */
    float    obs_return_deg;      /* 第 5 步：转回循迹线的角度               */
    float    obs_return_max_mm;   /* 第 6 步：斜着找线的最长距离             */
    float    obs_rejoin_fwd_mm;   /* 第 7 步：看到线后再走多远（≈探头到轮轴距离）*/
    float    obs_verify_mm;       /* 回线验证窗：看到线后前进此距离，线仍在才切循迹（防黑字误判）*/
    float    w_arc;               /* 圆弧兜圈的角速度（rad/s），半径 = v_avoid / w_arc        */
    float    w_arc_back;          /* 回程弧角速度（rad/s），比 w_arc 小 = 半径更大更平缓      */
    float    arc_out_deg;         /* 圆弧兜圈向外荡出的航向角（度）                          */
    float    arc_rejoin_deg;      /* （已弃用：兜圈认线不再设航向门槛）      */
    float    obs_state_timeout_s; /* 绕行中单个步骤超时，交还循迹            */
    float    v_rejoin;            /* 回线后的限速                            */
    float    rejoin_mm;           /* 回线后限速持续的距离                    */

    /* ---------- 超声波 ---------- */
    uint16_t us_slot_ms;          /* 每个超声波测量时间片                    */
    uint16_t us_max_mm;           /* 超时或超量程时的读数                    */

    /* ---------- 测试模式 ---------- */
    int16_t  test_duty;           /* TEST_MOTOR 开环占空比                   */
    float    test_speed;          /* TEST_STRAIGHT 速度                      */
    float    test_dist_mm;        /* TEST_STRAIGHT 距离                      */
    float    avoid_test_mm;       /* TEST_AVOID 总里程（走满即刹车）         */
} AppParam_t;

extern AppParam_t g_param;

#endif /* APP_CONFIG_H */
