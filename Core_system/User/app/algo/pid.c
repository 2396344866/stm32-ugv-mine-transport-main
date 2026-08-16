/**
  * @file    pid.c
  * @brief   串级 PID 控制器
  *
  *  外环：姿态(角度) 位置式 PD —— 输出速度修正量
  *  内环：速度 增量式 PI —— 输出 PWM 增量
  *  （参数取自井下轨道运输车控制系统设计：速度环 Kp=12 Ki=0.5，姿态环 Kp=5 Kd=2）
  */
#include "pid.h"

void PID_Init(PID_Handle* p, float Kp, float Ki, float Kd,
              float out_max, float out_min, float integ_limit)
{
    p->Kp = Kp; p->Ki = Ki; p->Kd = Kd;
    p->target = 0.0f;
    p->err_last = 0.0f; p->err_prev = 0.0f; p->integral = 0.0f;
    p->out_max = out_max; p->out_min = out_min;
    p->integ_limit = integ_limit;
    p->mode = 1;
}

void PID_SetTarget(PID_Handle* p, float target) { p->target = target; }
void PID_SetGains(PID_Handle* p, float Kp, float Ki, float Kd) { p->Kp = Kp; p->Ki = Ki; p->Kd = Kd; }

void PID_Reset(PID_Handle* p)
{
    p->err_last = 0.0f; p->err_prev = 0.0f; p->integral = 0.0f;
}

float PID_Calc_Position(PID_Handle* p, float meas)
{
    float error = p->target - meas;
    p->integral += error;
    if (p->integral >  p->integ_limit) p->integral =  p->integ_limit;
    if (p->integral < -p->integ_limit) p->integral = -p->integ_limit;

    float derivative = error - p->err_last;
    float output = p->Kp * error + p->Ki * p->integral + p->Kd * derivative;
    p->err_last = error;

    if (output > p->out_max) output = p->out_max;
    if (output < p->out_min) output = p->out_min;
    return output;
}

float PID_Calc_Incremental(PID_Handle* p, float meas)
{
    float error = p->target - meas;
    float inc = p->Kp * (error - p->err_last)
              + p->Ki * error
              + p->Kd * (error - 2.0f * p->err_last + p->err_prev);
    p->err_prev = p->err_last;
    p->err_last = error;
    return inc;
}
