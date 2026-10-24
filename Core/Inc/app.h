#ifndef APP_H
#define APP_H

#include <stdint.h>

typedef enum
{
    APP_WAIT = 0,   /* 上电等待出发     */
    APP_RUN,        /* 运行中           */
    APP_FINISHING,  /* 已过终点线，刹车 */
    APP_DONE,       /* 结束             */
    APP_FAULT       /* 故障停车         */
} AppState_t;

/* 调试观察用：在 STM32CubeIDE 的 Live Expressions 里添加 g_dbg 即可实时查看 */
typedef struct
{
    uint8_t  app_state;     /* AppState_t                         */
    uint8_t  line_status;   /* LineStatus_t                       */
    uint8_t  obs_state;     /* ObsState_t                         */
    uint8_t  obs_count;     /* 已完成的避障次数                   */
    int8_t   obs_dir;       /* +1 左绕  -1 右绕                   */
    uint8_t  track_alive;   /* 循迹模块是否在线                   */
    uint8_t  track_bits;    /* bit0 = 最左探头，1 = 压黑线        */
    uint8_t  black_cnt;
    float    line_pos;      /* 线位置 mm，右为正                  */
    uint32_t track_frames;
    uint16_t us_f, us_l, us_r;  /* 超声波距离 mm（滤波后）        */
    float    spd_l, spd_r;      /* 实测轮速 mm/s                  */
    float    tgt_l, tgt_r;      /* 目标轮速 mm/s                  */
    int16_t  duty_l, duty_r;    /* 占空比（千分比）               */
    float    v_cmd, w_cmd;
    float    odom;              /* 里程 mm                        */
    float    heading_deg;       /* 航向 度，左转为正              */
    uint32_t run_ms;            /* 本轮运行时间                   */
} AppDebug_t;

extern AppDebug_t g_dbg;

void App_Init(void);   /* 放在 main.c 的 USER CODE BEGIN 2 */
void App_Loop(void);   /* 放在 main.c 的 while(1) 里       */

#endif
