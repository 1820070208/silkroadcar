#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

typedef enum
{
    CH_OFF = 0,   /* 滑行             */
    CH_BRAKE,     /* 短路刹车         */
    CH_SPEED,     /* 速度闭环（v, w） */
    CH_OPEN       /* 开环占空比       */
} ChassisMode_t;

typedef struct
{
    ChassisMode_t mode;
    float   v_cmd, w_cmd;   /* 指令：线速度 mm/s，角速度 rad/s（左转为正） */
    float   v_ramp;         /* 加减速限制后的线速度                         */
    float   tgt_l, tgt_r;   /* 左右轮目标速度 mm/s                          */
    int16_t duty_l, duty_r; /* 实际输出占空比（千分比）                     */
} ChassisState_t;

void                  Chassis_Init(void);
void                  Chassis_SetVW(float v, float w);
void                  Chassis_SetOpen(int16_t duty_l, int16_t duty_r);
void                  Chassis_Stop(void);
void                  Chassis_Brake(void);
void                  Chassis_Update(float dt);   /* 在 Encoder_Update 之后调用 */
const ChassisState_t *Chassis_Get(void);

/* 航向保持：返回让车头朝向 target 的角速度 */
float   Chassis_HeadingW(float target_rad);
/* 原地转：输出角速度，到位返回 1 */
uint8_t Chassis_PivotW(float target_rad, float *w);

#endif
