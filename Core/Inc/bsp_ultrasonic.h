#ifndef BSP_ULTRASONIC_H
#define BSP_ULTRASONIC_H

#include <stdint.h>

typedef enum
{
    US_FRONT = 0,   /* 正前   */
    US_LEFT,        /* 左斜前 */
    US_RIGHT,       /* 右斜前 */
    US_NUM
} UsId_t;

void     Us_Init(void);
void     Us_Process(uint32_t now_ms);   /* 主循环里尽量频繁地调用 */
uint16_t Us_GetMm(UsId_t id);           /* 3 点中值滤波后的距离 mm */
uint16_t Us_GetRawMm(UsId_t id);        /* 最近一次原始距离 mm     */
uint32_t Us_GetSeq(UsId_t id);          /* 每出一个新读数加 1      */

#endif
