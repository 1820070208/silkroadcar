#include "pid.h"

static float clampf(float x, float lim)
{
    if (x > lim)  return lim;
    if (x < -lim) return -lim;
    return x;
}

void Pid_Reset(Pid_t *p)
{
    p->integ    = 0.0f;
    p->prev_err = 0.0f;
    p->first    = 1;
}

float Pid_Update(Pid_t *p, float err, float dt)
{
    float d = 0.0f;

    p->integ = clampf(p->integ + p->ki * err * dt, p->i_lim);

    if (!p->first && dt > 0.0f)
        d = (err - p->prev_err) / dt;
    p->first    = 0;
    p->prev_err = err;

    return clampf(p->kp * err + p->integ + p->kd * d, p->out_lim);
}
