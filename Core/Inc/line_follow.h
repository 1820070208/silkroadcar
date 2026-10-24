#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

typedef enum
{
    LINE_OK = 0,   /* 正常循迹                         */
    LINE_WIDE,     /* 宽线（起点/终点线），按航向直行  */
    LINE_LOST,     /* 刚丢线，保持上一次指令           */
    LINE_SEARCH,   /* 原地朝丢线方向转着找线           */
    LINE_FAIL      /* 找线超时                         */
} LineStatus_t;

void         Line_Reset(void);
/* 设定丢线后找线方向：+1 线在右边，-1 线在左边 */
void         Line_SetLastSide(int8_t side);
/* v_cruise：本周期允许的最高速度；输出 v(mm/s)、w(rad/s) */
LineStatus_t Line_Update(float dt, float v_cruise, float *v, float *w);

#endif
