#ifndef SAFETY_FSM_H
#define SAFETY_FSM_H

#include "stm32f10x.h"

/* ============================================================================
 * 多级安全状态机（控制子系统核心保护）
 *
 * 设计要点（工业界通用做法）：
 *  1. 三级量化保护：WARN(可恢复降速) / BRAKE / ERROR(锁死)；
 *  2. BRAKE / ERROR 为不可逆锁定态，需人工显式清除，避免阈值附近抖动反复进出；
 *  3. **N-of-M 越限确认**：单一采样点越限不算故障，必须 M 拍中至少 N 拍越限
 *     才升级 —— 抑制传感器单点尖峰 / DMP 跳变造成的误动作；
 *  4. **迟滞回差**：进入阈值与退出阈值分离（exit = enter*(1-hyst)），
 *     防止量值在阈值附近抖动导致状态反复翻转；
 *  5. **失效保持（fail-safe）**：传感器失效时按最保守值处理（视为越上限），
 *     而不是当作 0 放行。
 * ==========================================================================*/

typedef enum
{
    SAFE_RUN   = 0,
    SAFE_WARN  = 1,
    SAFE_BRAKE = 2,   /* 锁定：需显式清除 */
    SAFE_ERROR = 3    /* 锁定：需显式清除 */
} SafetyState;

/* N-of-M 确认器：连续 M 拍中至少 N 拍越限才置位 */
typedef struct
{
    uint8_t N;            /* 触发所需越限拍数 */
    uint8_t M;            /* 统计窗口拍数 */
    uint8_t cnt;          /* 当前窗口内越限计数 */
    uint8_t win;          /* 当前窗口已累计拍数 */
    uint8_t trip;         /* 1 = 已确认越限 */
} NomM;

/* 迟滞比较器：enter 触发 / exit 退出 */
typedef struct
{
    float  enter;         /* 进入故障的阈值幅值（对称 ±） */
    float  hyst;          /* 回差比例 0..1，exit = enter*(1-hyst) */
    NomM   nm;            /* 附带 N-of-M 确认 */
} HystTrip;

/* 传感器失效标志：1 = 本拍对应数据不可信 */
typedef struct
{
    uint8_t speed_bad;     /* 编码器测速失效 */
    uint8_t angle_bad;     /* 姿态解算失效 */
    uint8_t tension_bad;   /* 张力采样失效 */
} SafetyInval;

typedef struct
{
    SafetyState state;
    float target_speed;
    float warn_speed;
    float max_speed;
    float crit_angle;     /* 度 */
    float max_tension;    /* N */
    float min_tension;    /* N */

    /* N-of-M 确认参数 */
    uint8_t nm_n;         /* 触发所需越限拍数 */
    uint8_t nm_m;         /* 窗口拍数 */

    /* 滞回阈值（含 enter/exit 分离）与确认器 */
    HystTrip t_speed_max;     /* 速度上限 */
    HystTrip t_angle;         /* 倾角越限 */
    HystTrip t_tension_hi;    /* 张力上限 */
    HystTrip t_tension_lo;    /* 张力下限（单向低侧） */
} SafetyFSM;

/* 默认 N-of-M 与回差系数 */
#define SAFETY_NOM_N          (3u)    /* 连续 5 拍中 3 拍越限才动作 */
#define SAFETY_NOM_M          (5u)
#define SAFETY_HYST_SPEED     (0.10f) /* 速度回差 10%：120 进 / 108 出 */
#define SAFETY_HYST_ANGLE     (0.10f) /* 倾角回差 10%：25° 进 / 22.5° 出 */
#define SAFETY_HYST_TENSION   (0.05f) /* 张力回差 5%          */

void Safety_Init(SafetyFSM* s, float target_speed, float warn_speed, float max_speed,
                 float crit_angle, float max_tension, float min_tension);
/* inv 为 NULL 时全部视为有效；非 NULL 时 1 标志位表示该传感器本拍失效，按
 * fail-safe 处理（视为越上限，经 N-of-M 确认后升锁）。 */
SafetyState Safety_Update(SafetyFSM* s, float speed, float angle, float tension,
                          const SafetyInval* inv);
void Safety_ClearError(SafetyFSM* s);   /* 手动/看门狗复位后清除锁定 */

/* 供测试与 HMI 查询：某个保护点当前是否已确认越限 */
uint8_t Safety_Tripped(const HystTrip* t);

#endif /* SAFETY_FSM_H */
