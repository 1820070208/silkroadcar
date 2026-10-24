#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float   kp, ki, kd;
    float   i_lim;     /* 积分项限幅 */
    float   out_lim;   /* 输出限幅   */
    float   integ;
    float   prev_err;
    uint8_t first;
} Pid_t;

void  Pid_Reset(Pid_t *p);
float Pid_Update(Pid_t *p, float err, float dt);

#endif
