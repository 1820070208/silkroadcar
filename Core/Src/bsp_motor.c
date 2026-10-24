/**
 * bsp_motor.c  TB6612 双路电机驱动
 * 左电机：PWMA = TIM2_CH1(PA0)，AIN1 = PB12，AIN2 = PB13
 * 右电机：PWMB = TIM2_CH2(PA1)，BIN1 = PB14，BIN2 = PB15
 */
#include "bsp_motor.h"
#include "app_config.h"

static void motor_one(GPIO_TypeDef *p1, uint16_t n1, GPIO_TypeDef *p2, uint16_t n2,
                      uint32_t ch, int32_t duty, uint8_t reverse)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&HW_PWM_TIM);
    uint32_t mag;

    if (reverse) duty = -duty;
    mag = (uint32_t)(duty >= 0 ? duty : -duty);
    if (mag > 1000u) mag = 1000u;

    if (duty > 0)
    {
        HAL_GPIO_WritePin(p1, n1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(p2, n2, GPIO_PIN_RESET);
    }
    else if (duty < 0)
    {
        HAL_GPIO_WritePin(p1, n1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(p2, n2, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(p1, n1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(p2, n2, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIM, ch, (arr + 1u) * mag / 1000u);
}

void Motor_Init(void)
{
    Motor_Coast();
    HAL_TIM_PWM_Start(&HW_PWM_TIM, HW_PWM_CH_L);
    HAL_TIM_PWM_Start(&HW_PWM_TIM, HW_PWM_CH_R);
}

void Motor_Set(int16_t duty_l, int16_t duty_r)
{
    motor_one(HW_AIN1_PORT, HW_AIN1_PIN, HW_AIN2_PORT, HW_AIN2_PIN,
              HW_PWM_CH_L, duty_l, CFG_MOTOR_L_REVERSE);
    motor_one(HW_BIN1_PORT, HW_BIN1_PIN, HW_BIN2_PORT, HW_BIN2_PIN,
              HW_PWM_CH_R, duty_r, CFG_MOTOR_R_REVERSE);
}

void Motor_Brake(void)
{
    HAL_GPIO_WritePin(HW_AIN1_PORT, HW_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HW_AIN2_PORT, HW_AIN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HW_BIN1_PORT, HW_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HW_BIN2_PORT, HW_BIN2_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIM, HW_PWM_CH_L, 0);
    __HAL_TIM_SET_COMPARE(&HW_PWM_TIM, HW_PWM_CH_R, 0);
}

void Motor_Coast(void)
{
    Motor_Set(0, 0);
}
