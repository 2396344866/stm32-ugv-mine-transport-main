#ifndef PID_H
#define PID_H

#include "stm32f10x.h"

typedef struct
{
    float Kp, Ki, Kd;
    float target;
    float err_last;     /* e(k-1) */
    float err_prev;     /* e(k-2) */
    float integral;
    float integ_limit;  /* 积分限幅（抗饱和） */
    float out_max;
    float out_min;
    uint8_t mode;       /* 0=位置式, 1=增量式 */
} PID_Handle;

void PID_Init(PID_Handle* p, float Kp, float Ki, float Kd,
              float out_max, float out_min, float integ_limit);
void PID_SetTarget(PID_Handle* p, float target);
void PID_SetGains(PID_Handle* p, float Kp, float Ki, float Kd);
void PID_Reset(PID_Handle* p);

/* 位置式 PID（用于外环姿态） */
float PID_Calc_Position(PID_Handle* p, float meas);
/* 增量式 PID（用于内环速度），返回增量，由调用方累加 */
float PID_Calc_Incremental(PID_Handle* p, float meas);

#endif /* PID_H */
