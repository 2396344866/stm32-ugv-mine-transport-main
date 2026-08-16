#ifndef SAFETY_FSM_H
#define SAFETY_FSM_H

#include "stm32f10x.h"

typedef enum
{
    SAFE_RUN   = 0,
    SAFE_WARN  = 1,
    SAFE_BRAKE = 2,   /* 锁定：需显式清除 */
    SAFE_ERROR = 3    /* 锁定：需显式清除 */
} SafetyState;

/* 多级安全状态机（控制子系统核心保护）
 * ERROR/BRAKE 为不可逆锁定态，避免抖动误恢复。 */
typedef struct
{
    SafetyState state;
    float target_speed;
    float warn_speed;
    float max_speed;
    float crit_angle;     /* 度 */
    float max_tension;    /* N */
    float min_tension;    /* N */
} SafetyFSM;

void Safety_Init(SafetyFSM* s, float target_speed, float warn_speed, float max_speed,
                 float crit_angle, float max_tension, float min_tension);
SafetyState Safety_Update(SafetyFSM* s, float speed, float angle, float tension);
void Safety_ClearError(SafetyFSM* s);   /* 手动/看门狗复位后清除锁定 */

#endif /* SAFETY_FSM_H */
