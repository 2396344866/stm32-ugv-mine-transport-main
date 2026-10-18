/**
 * @file    safety_fsm.c
 * @brief   多级安全状态机（N-of-M 越限确认 + 迟滞回差 + 失效保持）
 *
 *  状态转移（仅当未处于锁定态时评估）：
 *   拉力越限 / <下限        -> ERROR（锁死，电机停）
 *   姿态角越限              -> BRAKE（锁死，电磁制动）
 *   速度越上限              -> BRAKE
 *   速度越预警限            -> WARN（可恢复）
 *   否则                    -> RUN
 *
 *  三重抗误动措施：
 *   ① N-of-M：M 拍中至少 N 拍越限才升级，单点尖峰/DMP 跳变不再直接锁死；
 *   ② 迟滞回差：enter 与 exit 分离（回差 = enter × hyst），消除阈值抖动翻转；
 *   ③ 失效保持：传感器失效按最保守值处理，不当作 0 放行。
 *
 *  注：BRAKE/ERROR 一旦进入即锁死，仍需 HMI 显式清除（Safety_ClearError）；
 *  故 N-of-M 采用「滑动窗口」而非「连续 N 拍」，以免长时间异常工况下计数
 *  被偶发正常拍打断而永远不动作。
 */
#include "safety_fsm.h"
#include <stddef.h>

static float f_abs(float x) { return (x < 0.0f) ? -x : x; }

/* 初始化 N-of-M 确认器 */
static void nom_init(NomM* nm, uint8_t n, uint8_t m)
{
    nm->N    = n;
    nm->M    = m;
    nm->cnt  = 0;
    nm->win  = 0;
    nm->trip = 0;
}

/* 初始化迟滞保护点 */
static void trip_init(HystTrip* t, float enter, float hyst, uint8_t n, uint8_t m)
{
    t->enter = enter;
    t->hyst  = hyst;
    nom_init(&t->nm, n, m);
}

/**
 * 喂一拍越限标志（1 = 本拍量值越过 enter/exit 滞回判定线）
 * 返回 1 = 该保护点已确认越限（M 拍窗口内累计 ≥ N 拍）
 * 越限线本身由调用方用 hyst_over()/hyst_under() 给出（已含 enter/exit 分离）。
 */
static uint8_t trip_update(NomM* nm, uint8_t over)
{
    if (over)
    {
        /* 已确认越限时锁存计数，避免窗口滚动把 trip 冲掉 */
        if (nm->trip)
        {
            nm->cnt = nm->M;
            return 1;
        }
        if (nm->cnt < nm->M) nm->cnt++;
        if (nm->win < nm->M) nm->win++;
        if (nm->cnt >= nm->N) nm->trip = 1;
    }
    else
    {
        /* 低于 exit 线：滑动窗口左移一格（丢弃最老一拍） */
        if (nm->win > 0) nm->win--;
        if (nm->cnt > 0) nm->cnt--;
        if (nm->cnt < nm->N) nm->trip = 0;
    }
    return nm->trip;
}

/**
 * 迟滞比较：mag 为待判量幅值（对称量），返回 1 = 处于故障侧
 *  - 未触发时：mag >= enter 判故障
 *  - 已触发时：mag <  enter*(1-hyst) 才判恢复
 */
static uint8_t hyst_over(const HystTrip* t, float mag)
{
    if (t->nm.trip)
    {
        return (mag >= (t->enter * (1.0f - t->hyst))) ? 1u : 0u;
    }
    return (mag >= t->enter) ? 1u : 0u;
}

/* 低侧单向保护：低于 min_t 判故障，恢复需回到 min_t*(1+hyst) 以上 */
static uint8_t hyst_under(const HystTrip* t, float mag)
{
    if (t->nm.trip)
    {
        return (mag <= (t->enter * (1.0f + t->hyst))) ? 1u : 0u;
    }
    return (mag <= t->enter) ? 1u : 0u;
}

uint8_t Safety_Tripped(const HystTrip* t)
{
    return (t != NULL) ? t->nm.trip : 0u;
}

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

    s->nm_n = SAFETY_NOM_N;
    s->nm_m = SAFETY_NOM_M;

    trip_init(&s->t_speed_max,  max_speed,    SAFETY_HYST_SPEED,   s->nm_n, s->nm_m);
    trip_init(&s->t_angle,      crit_angle,   SAFETY_HYST_ANGLE,   s->nm_n, s->nm_m);
    trip_init(&s->t_tension_hi, max_tension,  SAFETY_HYST_TENSION, s->nm_n, s->nm_m);
    trip_init(&s->t_tension_lo, min_tension,  SAFETY_HYST_TENSION, s->nm_n, s->nm_m);
}

SafetyState Safety_Update(SafetyFSM* s, float speed, float angle, float tension,
                          const SafetyInval* inv)
{
    uint8_t sp_bad = (inv != NULL) ? inv->speed_bad   : 0u;
    uint8_t an_bad = (inv != NULL) ? inv->angle_bad   : 0u;
    uint8_t te_bad = (inv != NULL) ? inv->tension_bad : 0u;

    /* fail-safe：传感器失效时用 INT32 哨兵值喂入比较，必然越过 enter 阈值，
     * 再经 N-of-M 确认后升锁 —— 绝不允许「读不到 = 0 = 正常」这种失效放行。 */
    const float SENTINEL = 1.0e9f;
    float sp = sp_bad ? SENTINEL : f_abs(speed);
    float an = an_bad ? SENTINEL : f_abs(angle);
    float te_hi = te_bad ? SENTINEL : tension;
    float te_lo = te_bad ? -SENTINEL : tension;

    /* 逐保护点评估：先用带滞回的比较器给出「本拍是否在故障侧」，
     * 再送入 N-of-M 确认器累计。 */
    uint8_t sp_trip  = trip_update(&s->t_speed_max.nm,  (uint8_t)(hyst_over(&s->t_speed_max, sp)  || sp_bad));
    uint8_t an_trip  = trip_update(&s->t_angle.nm,      (uint8_t)(hyst_over(&s->t_angle, an)      || an_bad));
    uint8_t te_hi_t  = trip_update(&s->t_tension_hi.nm, (uint8_t)(hyst_over(&s->t_tension_hi, te_hi) || te_bad));
    uint8_t te_lo_t  = trip_update(&s->t_tension_lo.nm, (uint8_t)(hyst_under(&s->t_tension_lo, te_lo) || te_bad));

    /* 锁定态不再评估，避免抖动导致反复进出刹车 */
    if (s->state == SAFE_ERROR || s->state == SAFE_BRAKE)
    {
        return s->state;
    }

    if (te_hi_t || te_lo_t)
    {
        s->state = SAFE_ERROR;
    }
    else if (an_trip || sp_trip)
    {
        s->state = SAFE_BRAKE;
    }
    else if (sp >= s->warn_speed)
    {
        s->state = SAFE_WARN;
    }
    else
    {
        s->state = SAFE_RUN;
    }
    return s->state;
}

void Safety_ClearError(SafetyFSM* s)
{
    s->state = SAFE_RUN;
    /* 人工确认安全后复位全部越限确认器，避免残留计数立刻二次触发 */
    nom_init(&s->t_speed_max.nm,  s->nm_n, s->nm_m);
    nom_init(&s->t_angle.nm,      s->nm_n, s->nm_m);
    nom_init(&s->t_tension_hi.nm, s->nm_n, s->nm_m);
    nom_init(&s->t_tension_lo.nm, s->nm_n, s->nm_m);
}
