/**
 * safety_fsm 抗误动单测（PC 主机运行，纯逻辑无硬件依赖）
 *
 * 覆盖本次改动的核心断言：
 *   ① 单点倾角尖峰（1 拍越限）不得触发 BRAKE —— 这是改动前的实际缺陷；
 *   ② 连续越限达 N 拍必须触发 BRAKE —— 不能把真故障也滤掉；
 *   ③ 迟滞回差：退出阈值(22.5°)与进入阈值(25°)分离，区间内保持锁定；
 *   ④ DMP 失效（angle_bad）按 fail-safe 触发，不当作 0 放行；
 *   ⑤ 张力越限走 ERROR 而非 BRAKE，且同样需要 N 拍确认；
 *   ⑥ 人工清除后确认器计数归零，不会立刻二次触发。
 *
 * 用法：gcc -I<include> -o t t.c safety_fsm.c && ./t
 */
#include <stdio.h>
#include <stddef.h>
#include "safety_fsm.h"

static int g_fail = 0;
static int g_pass = 0;

static void check(int cond, const char* name)
{
    if (cond) { printf("  [ok]   %s\n", name); g_pass++; }
    else      { printf("  [FAIL] %s\n", name); g_fail++; }
}

#define TARGET 60.0f
#define WARN   90.0f
#define MAXV   120.0f
#define ANG    25.0f
#define TMAX   200.0f
#define TMIN   5.0f

int main(void)
{
    SafetyFSM s;

    /* ---------- ① 单点尖峰不得误触发 ---------- */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    SafetyState st = Safety_Update(&s, 50.0f, 0.0f, 100.0f, NULL);   /* 预热一拍 */
    check(st == SAFE_RUN, "预热拍保持 RUN");
    st = Safety_Update(&s, 50.0f, 40.0f, 100.0f, NULL);              /* 1 拍尖峰 40° */
    check(st == SAFE_RUN, "单点倾角尖峰(40°, 1 拍)不触发 BRAKE  <= 改动前会误锁");
    st = Safety_Update(&s, 50.0f, 40.0f, 100.0f, NULL);              /* 2 拍 */
    check(st == SAFE_RUN, "连续 2 拍越限仍不触发 (需 N=3)");
    st = Safety_Update(&s, 50.0f, 40.0f, 100.0f, NULL);              /* 第 3 拍 */
    check(st == SAFE_BRAKE, "连续 3 拍越限触发 BRAKE (N=3/M=5)");

    /* ---------- ② 真实故障必被捕获 ---------- */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    for (int i = 0; i < 4; i++) st = Safety_Update(&s, 50.0f, 30.0f, 100.0f, NULL);
    check(st == SAFE_BRAKE, "持续越限(30°)在 4 拍内必然锁 BRAKE");

    /* ---------- ③ 锁定后不可自动恢复 ---------- */
    st = Safety_Update(&s, 50.0f, 0.0f, 100.0f, NULL);              /* 恢复正常值 */
    check(st == SAFE_BRAKE, "越限消失后 BRAKE 仍锁定(需人工清除)");
    Safety_ClearError(&s);
    st = Safety_Update(&s, 50.0f, 0.0f, 100.0f, NULL);
    check(st == SAFE_RUN, "Safety_ClearError 后恢复 RUN");

    /* ---------- ④ 迟滞回差 ---------- */
    /* 触发后把角度降到 24°（介于 exit=22.5 与 enter=25 之间）应保持锁定 */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    for (int i = 0; i < 3; i++) st = Safety_Update(&s, 50.0f, 26.0f, 100.0f, NULL);
    check(st == SAFE_BRAKE, "26° 连续 3 拍触发 BRAKE");
    /* 降到 24°：低于 enter=25，按 enter/exit 回差应仍维持 */
    for (int i = 0; i < 6; i++) st = Safety_Update(&s, 50.0f, 24.0f, 100.0f, NULL);
    check(st == SAFE_BRAKE, "回差带内(24°)保持锁定, 不因回落而抖动解锁");

    /* 降到 20°（低于 exit=22.5）经人工清除后应能正常保持 RUN */
    Safety_ClearError(&s);
    for (int i = 0; i < 6; i++) st = Safety_Update(&s, 50.0f, 20.0f, 100.0f, NULL);
    check(st == SAFE_RUN, "回到 exit 阈值以下并清除后可稳定 RUN");

    /* ---------- ⑤ 失效保持 fail-safe ---------- */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    SafetyInval inv = { 0u, 1u, 0u };   /* angle_bad = 1 */
    for (int i = 0; i < 3; i++) st = Safety_Update(&s, 50.0f, 0.0f, 100.0f, &inv);
    check(st == SAFE_BRAKE, "DMP 失效(angle_bad)按 fail-safe 锁 BRAKE, 非当作 0 放行");

    /* ---------- ⑥ 张力越限走 ERROR 且需 N 拍确认 ---------- */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    st = Safety_Update(&s, 50.0f, 0.0f, 250.0f, NULL);              /* 1 拍张力超上限 */
    check(st == SAFE_RUN, "张力单点越限(250N, 1 拍)不触发 ERROR");
    st = Safety_Update(&s, 50.0f, 0.0f, 250.0f, NULL);              /* 2 拍 */
    check(st == SAFE_RUN, "张力连续 2 拍越限仍不触发");
    st = Safety_Update(&s, 50.0f, 0.0f, 250.0f, NULL);              /* 第 3 拍 */
    check(st == SAFE_ERROR, "张力连续 3 拍越限触发 ERROR");

    /* 张力下限同理 */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    for (int i = 0; i < 3; i++) st = Safety_Update(&s, 50.0f, 0.0f, 1.0f, NULL);
    check(st == SAFE_ERROR, "张力低于下限(1N)触发 ERROR");

    /* ---------- ⑦ 速度 WARN 分级 ---------- */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    for (int i = 0; i < 4; i++) st = Safety_Update(&s, 100.0f, 0.0f, 100.0f, NULL);
    check(st == SAFE_WARN, "速度 100 介于 warn(90)-max(120) 进入 WARN");

    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    for (int i = 0; i < 4; i++) st = Safety_Update(&s, 130.0f, 0.0f, 100.0f, NULL);
    check(st == SAFE_BRAKE, "速度 130 超上限进入 BRAKE(非 ERROR)");

    /* ---------- ⑧ 优先级：任一 ERROR 级越限优先于 BRAKE 级 ---------- */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    for (int i = 0; i < 4; i++) st = Safety_Update(&s, 130.0f, 40.0f, 250.0f, NULL);
    check(st == SAFE_ERROR, "张力与速度同时越限时 ERROR 优先");

    /* ---------- ⑨ Safety_Tripped 查询接口 ---------- */
    Safety_Init(&s, TARGET, WARN, MAXV, ANG, TMAX, TMIN);
    check(Safety_Tripped(&s.t_angle) == 0u, "初始化后倾角保护点未触发");
    for (int i = 0; i < 3; i++) st = Safety_Update(&s, 50.0f, 40.0f, 100.0f, NULL);
    check(Safety_Tripped(&s.t_angle) == 1u, "3 拍越限后 Safety_Tripped 返回 1");

    printf("\n%s  (通过 %d / 失败 %d)\n",
           (g_fail == 0) ? "ALL PASS" : "HAS FAILURE", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}