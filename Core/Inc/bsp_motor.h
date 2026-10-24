#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include <stdint.h>

void Motor_Init(void);
/* 占空比千分比，-1000 ~ 1000，正为前进 */
void Motor_Set(int16_t duty_l, int16_t duty_r);
/* 短路刹车（TB6612 IN1 = IN2 = H） */
void Motor_Brake(void);
/* 滑行（IN1 = IN2 = L） */
void Motor_Coast(void);

#endif
