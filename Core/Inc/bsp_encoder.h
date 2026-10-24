#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

typedef struct
{
    int32_t cnt_l, cnt_r;      /* 累计计数                         */
    float   speed_l, speed_r;  /* 轮速 mm/s，前进为正              */
    float   dist_l, dist_r;    /* 单轮累计距离 mm                  */
    float   odom;              /* 车体中心累计前进距离 mm          */
    float   heading;           /* 航向 rad，左转为正，不做 ±π 回绕 */
} Odom_t;

void          Encoder_Init(void);
void          Encoder_Update(float dt);   /* 每个控制周期调用一次 */
void          Encoder_ResetOdom(void);
const Odom_t *Encoder_Get(void);

#endif
