#ifndef ALARM_H
#define ALARM_H

#include "stm32f10x.h"

typedef enum
{
    ALARM_NORMAL = 0,
    ALARM_WARN   = 1,
    ALARM_CRIT   = 2
} AlarmLevel;

/* 迟滞报警状态机：进入阈值宽、退出阈值窄，避免临界点抖动 */
typedef struct
{
    float ref;          /* 参考值（例如标称气压） */
    float warn_enter;   /* 偏差 >= 该比例进入预警（如 0.05） */
    float warn_exit;    /* 偏差 <= 该比例退出预警（如 0.02，迟滞带） */
    float crit_enter;   /* 偏差 >= 该比例进入严重（如 0.08） */
    AlarmLevel level;
} AlarmHysteresis;

void Alarm_Init(AlarmHysteresis* a, float ref, float warn_enter, float warn_exit, float crit_enter);
/* 返回当前报警等级（带迟滞） */
AlarmLevel Alarm_Update(AlarmHysteresis* a, float meas);
float Alarm_DeviationRatio(float ref, float meas);   /* |meas-ref|/ref */

#endif /* ALARM_H */
