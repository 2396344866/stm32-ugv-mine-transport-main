/**
  * @file    filter.c
  * @brief   混合数字滤波
  */
#include "filter.h"

void Filter_Init(Filter_Handle* f, float alpha)
{
    for (uint8_t i = 0; i < 8; i++) f->buf[i] = 0.0f;
    f->idx = 0; f->cnt = 0; f->lag_out = 0.0f; f->alpha = alpha;
}

float Filter_TrimmedMean(float* buf, uint8_t n)
{
    if (n == 0) return 0.0f;
    if (n == 1) return buf[0];
    float sum = 0.0f, maxv = buf[0], minv = buf[0];
    for (uint8_t i = 0; i < n; i++)
    {
        sum += buf[i];
        if (buf[i] > maxv) maxv = buf[i];
        if (buf[i] < minv) minv = buf[i];
    }
    /* 去一个最大、一个最小（n>=3 时） */
    if (n >= 3) sum = sum - maxv - minv;
    return sum / (float)((n >= 3) ? (n - 2) : n);
}

float Filter_FirstOrderLag(float prev, float x, float alpha)
{
    return prev + alpha * (x - prev);
}

float Filter_Update(Filter_Handle* f, float raw)
{
    f->buf[f->idx] = raw;
    f->idx = (f->idx + 1) % 8;
    if (f->cnt < 8) f->cnt++;

    float mean = Filter_TrimmedMean(f->buf, f->cnt);
    f->lag_out = Filter_FirstOrderLag(f->lag_out, mean, f->alpha);
    return f->lag_out;
}
