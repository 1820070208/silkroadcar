/**
 * bsp_ultrasonic.c  3 路 HC-SR04（GPIO 模式）
 *
 * 右超声波 PB8/PB9 属于 TIM4，而 TIM4 已用作编码器，无法做输入捕获；
 * 所以 3 路 ECHO 都用“双边沿外部中断 + TIM3 1MHz 自由计数”测脉宽。
 *   前：TRIG PA6  ECHO PA7(EXTI7)
 *   左：TRIG PB0  ECHO PB1(EXTI1)
 *   右：TRIG PB8  ECHO PB9(EXTI9)
 * 3 路轮流触发（前-左-前-右），同一时刻只有一路在测，避免串扰。
 */
#include "bsp_ultrasonic.h"
#include "app_config.h"

typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;
    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;
} UsHw_t;

enum { ST_IDLE = 0, ST_WAIT_RISE, ST_WAIT_FALL, ST_DONE };

static const UsHw_t s_hw[US_NUM] =
{
    { HW_US_F_TRIG_PORT, HW_US_F_TRIG_PIN, HW_US_F_ECHO_PORT, HW_US_F_ECHO_PIN },
    { HW_US_L_TRIG_PORT, HW_US_L_TRIG_PIN, HW_US_L_ECHO_PORT, HW_US_L_ECHO_PIN },
    { HW_US_R_TRIG_PORT, HW_US_R_TRIG_PIN, HW_US_R_ECHO_PORT, HW_US_R_ECHO_PIN },
};

/* 触发顺序：前方更新得更频繁 */
static const uint8_t s_seq[] = { US_FRONT, US_LEFT, US_FRONT, US_RIGHT };
#define SEQ_LEN  (sizeof(s_seq) / sizeof(s_seq[0]))

static volatile int8_t   s_active = -1;
static volatile uint8_t  s_state;
static volatile uint16_t s_t_rise;
static volatile uint16_t s_width_us;

static uint8_t  s_seq_idx;
static uint32_t s_slot_t0;
static uint8_t  s_taken;

static uint16_t s_hist[US_NUM][3];
static uint8_t  s_hist_idx[US_NUM];
static uint16_t s_raw[US_NUM];
static uint16_t s_filt[US_NUM];
static uint32_t s_cnt[US_NUM];

static inline uint16_t us_now(void)
{
    return (uint16_t)HW_US_TIM.Instance->CNT;
}

static uint16_t med3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) b = c;
    return (a > b) ? a : b;
}

static void us_push(uint8_t id, uint32_t mm)
{
    uint16_t *h;

    if (mm > g_param.us_max_mm) mm = g_param.us_max_mm;
    s_raw[id] = (uint16_t)mm;

    h = s_hist[id];
    h[s_hist_idx[id]] = (uint16_t)mm;
    s_hist_idx[id] = (uint8_t)((s_hist_idx[id] + 1u) % 3u);
    s_filt[id] = med3(h[0], h[1], h[2]);
    s_cnt[id]++;
}

static void us_trigger(uint8_t id)
{
    uint16_t t0;

    s_state  = ST_WAIT_RISE;
    s_active = (int8_t)id;

    HAL_GPIO_WritePin(s_hw[id].trig_port, s_hw[id].trig_pin, GPIO_PIN_SET);
    t0 = us_now();
    while ((uint16_t)(us_now() - t0) < 12u) { }
    HAL_GPIO_WritePin(s_hw[id].trig_port, s_hw[id].trig_pin, GPIO_PIN_RESET);
}

void Us_Init(void)
{
    uint8_t i, j;

    for (i = 0; i < US_NUM; i++)
    {
        HAL_GPIO_WritePin(s_hw[i].trig_port, s_hw[i].trig_pin, GPIO_PIN_RESET);
        for (j = 0; j < 3; j++) s_hist[i][j] = g_param.us_max_mm;
        s_raw[i]  = g_param.us_max_mm;
        s_filt[i] = g_param.us_max_mm;
    }
    HAL_TIM_Base_Start(&HW_US_TIM);
    s_seq_idx = SEQ_LEN - 1u;
    s_active  = -1;
}

void Us_Process(uint32_t now_ms)
{
    int8_t id = s_active;

    if (id >= 0)
    {
        if (!s_taken && s_state == ST_DONE)
        {
            /* 距离 = 脉宽(us) × 343m/s ÷ 2 = 脉宽 × 0.1715 mm */
            us_push((uint8_t)id, (uint32_t)s_width_us * 343u / 2000u);
            s_taken = 1;
        }
        if ((uint32_t)(now_ms - s_slot_t0) < g_param.us_slot_ms)
            return;
        if (!s_taken)
            us_push((uint8_t)id, g_param.us_max_mm);   /* 超时：视为前方无物 */
    }

    s_seq_idx = (uint8_t)((s_seq_idx + 1u) % SEQ_LEN);
    s_slot_t0 = now_ms;
    s_taken   = 0;
    us_trigger(s_seq[s_seq_idx]);
}

uint16_t Us_GetMm(UsId_t id)    { return s_filt[id]; }
uint16_t Us_GetRawMm(UsId_t id) { return s_raw[id]; }
uint32_t Us_GetSeq(UsId_t id)   { return s_cnt[id]; }

/* 3 路 ECHO 都是双边沿外部中断，由 HAL_GPIO_EXTI_IRQHandler 回调到这里 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint16_t t  = us_now();
    int8_t   id = s_active;

    if (id < 0 || GPIO_Pin != s_hw[id].echo_pin)
        return;

    if (HAL_GPIO_ReadPin(s_hw[id].echo_port, GPIO_Pin) == GPIO_PIN_SET)
    {
        if (s_state == ST_WAIT_RISE)
        {
            s_t_rise = t;
            s_state  = ST_WAIT_FALL;
        }
    }
    else if (s_state == ST_WAIT_FALL)
    {
        s_width_us = (uint16_t)(t - s_t_rise);
        s_state    = ST_DONE;
    }
}
