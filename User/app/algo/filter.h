#ifndef FILTER_H
#define FILTER_H

#include "stm32f10x.h"

/* 混合数字滤波：去极值平均 + 一阶滞后（工业管道气压监测） */
typedef struct
{
    float buf[8];
    uint8_t idx;
    uint8_t cnt;
    float lag_out;
    float alpha;     /* 一阶滞后系数 0..1，越大越平滑 */
} Filter_Handle;

void  Filter_Init(Filter_Handle* f, float alpha);
float Filter_TrimmedMean(float* buf, uint8_t n);          /* 去最大最小后平均 */
float Filter_FirstOrderLag(float prev, float x, float alpha);
float Filter_Update(Filter_Handle* f, float raw);          /* 去极值平均 -> 一阶滞后 */

#endif /* FILTER_H */
