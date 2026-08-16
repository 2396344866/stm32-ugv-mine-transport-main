/**
  * @file    safety_fsm.c
  * @brief   多级安全状态机
  *
  *  状态转移（仅当未处于锁定态时评估）：
  *   拉力越限 / <下限        -> ERROR（锁死，电机停）
  *   姿态角越限              -> BRAKE（锁死，电磁制动）
  *   速度越上限              -> BRAKE
  *   速度越预警限            -> WARN（可恢复）
  *   否则                    -> RUN
  */
#include "safety_fsm.h"

void Safety_Init(SafetyFSM* s, float target_speed, float warn_speed, float max_speed,
                 float crit_angle, float max_tension, float min_tension)
{
    s->state         = SAFE_RUN;
    s->target_speed  = target_speed;
    s->warn_speed    = warn_speed;
    s->max_speed     = max_speed;
    s->crit_angle    = crit_angle;
    s->max_tension   = max_tension;
    s->min_tension   = min_tension;
}

SafetyState Safety_Update(SafetyFSM* s, float speed, float angle, float tension)
{
    if (s->state != SAFE_ERROR && s->state != SAFE_BRAKE)
    {
        if (tension > s->max_tension || tension < s->min_tension)
        {
            s->state = SAFE_ERROR;
        }
        else if ((angle >  s->crit_angle) || (angle < -s->crit_angle))
        {
            s->state = SAFE_BRAKE;
        }
        else if (((speed > 0) ? speed : -speed) > s->max_speed)
        {
            s->state = SAFE_BRAKE;
        }
        else if (((speed > 0) ? speed : -speed) > s->warn_speed)
        {
            s->state = SAFE_WARN;
        }
        else
        {
            s->state = SAFE_RUN;
        }
    }
    return s->state;
}

void Safety_ClearError(SafetyFSM* s)
{
    s->state = SAFE_RUN;
}
