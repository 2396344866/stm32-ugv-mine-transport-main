/**
  * @file    alarm.c
  * @brief   迟滞报警状态机（±5% 预警 / ±2% 退出迟滞带 / 严重阈值）
  */
#include "alarm.h"

void Alarm_Init(AlarmHysteresis* a, float ref, float warn_enter, float warn_exit, float crit_enter)
{
    a->ref = ref;
    a->warn_enter = warn_enter;
    a->warn_exit  = warn_exit;
    a->crit_enter = crit_enter;
    a->level = ALARM_NORMAL;
}

float Alarm_DeviationRatio(float ref, float meas)
{
    if (ref == 0.0f) return 0.0f;
    return (meas - ref) / ref;
}

AlarmLevel Alarm_Update(AlarmHysteresis* a, float meas)
{
    float dev = Alarm_DeviationRatio(a->ref, meas);
    float adev = (dev < 0) ? -dev : dev;

    switch (a->level)
    {
        case ALARM_NORMAL:
            if (adev >= a->crit_enter)      a->level = ALARM_CRIT;
            else if (adev >= a->warn_enter) a->level = ALARM_WARN;
            break;
        case ALARM_WARN:
            if (adev >= a->crit_enter)      a->level = ALARM_CRIT;
            else if (adev <= a->warn_exit)  a->level = ALARM_NORMAL;
            break;
        case ALARM_CRIT:
            /* 严重告警需回落到预警带内才降级（迟滞） */
            if (adev <= a->warn_exit)       a->level = ALARM_NORMAL;
            else if (adev < a->crit_enter)  a->level = ALARM_WARN;
            break;
        default:
            a->level = ALARM_NORMAL;
            break;
    }
    return a->level;
}
