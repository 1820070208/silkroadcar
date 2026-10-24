#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <stdint.h>

typedef enum
{
    OBS_IDLE = 0,       /* 未检测到障碍                     */
    OBS_APPROACH,       /* 前方有物：减速接近，记录左右读数 */
    OBS_TURN_OUT,       /* 1. 原地转向空的一侧（旧三段式，保留未用） */
    OBS_SHIFT,          /* 2. 斜着走出 obs_shift_mm 横向距离（旧）  */
    OBS_TURN_PARALLEL,  /* 3. 转回原航向（旧）                      */
    OBS_PASS,           /* 4. 平行通过障碍物（旧，仅兜底路径可达）  */
    OBS_TURN_BACK,      /* 5. 转向循迹线（旧）                      */
    OBS_RETURN,         /* 6. 斜着走直到看到线（仍作兜底）          */
    OBS_REJOIN_FWD,     /* 7. 回线验证窗（20mm 占比判定）           */
    OBS_REJOIN_ALIGN,   /* 8. 转回原航向，交还循迹（旧）            */
    OBS_ARC_OUT,        /* 新·圆弧兜圈 1：恒速恒角速度向外荡出      */
    OBS_ARC_BACK        /* 新·圆弧兜圈 2：曲率反向往回兜，见线验证  */
} ObsState_t;

typedef enum
{
    OBS_OUT_NONE = 0,   /* 避障不干预                           */
    OBS_OUT_SLOW,       /* 继续循迹，但限速为输出的 v           */
    OBS_OUT_TAKEOVER    /* 避障接管底盘，直接使用输出的 v、w    */
} ObsOut_t;

void       Obs_Reset(void);
ObsOut_t   Obs_Update(float dt, float *v, float *w);
ObsState_t Obs_GetState(void);
uint8_t    Obs_GetCount(void);
int8_t     Obs_GetDir(void);     /* +1 从左侧绕，-1 从右侧绕 */

#endif
