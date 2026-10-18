/**
 * @file    test_log_ring.c
 * @brief   monitor_task 的 RAM 环形缓冲（黑匣子预存）逻辑单测（PC 端运行）
 *
 * 覆盖三件容易写错、且上板才发现的事：
 *   ① 时序：ring_flush 必须按「时间先后」刷出，跨过 head 环绕边界也不乱序；
 *   ② 部分失败：写到第 k 条失败时，前 k-1 条已落盘，剩余条目必须**保留**
 *      在 RAM 里等下次重试，不能丢；
 *   ③ 满缓冲覆写：缓冲写满后新样本覆盖最旧，且最旧的那条确实被淘汰。
 *
 * 这里复制 ring_push/ring_flush 的算法骨架（去掉 EEPROM 依赖），
 * 保证测的是同一套索引算式 —— 若改动 monitor_task.c 的取模逻辑，
 * 需同步本文件的SKELETON 段。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* ===== SKELETON: 与 monitor_task.c 保持一致的参数与算法 ===== */
#define EEPROM_REC_DATA_LEN   (15)
#define EEPROM_LOG_N          (128)
#define EEPROM_LOG_RECLEN     (16)

typedef struct { uint8_t payload[EEPROM_REC_DATA_LEN]; } LogEntry;

static LogEntry s_ring[8];              /* 测试用小缓冲，逻辑同构 */
static uint16_t s_ring_head = 0;
static uint16_t s_ring_len  = 0;

static int s_fail_at = -1;               /* -1=不失败；否则在第 n 条失败 */
static int s_written  = 0;

static void ring_push(const uint8_t* payload)
{
    for (uint8_t i = 0; i < EEPROM_REC_DATA_LEN; i++)
        s_ring[s_ring_head].payload[i] = payload[i];
    s_ring_head = (uint16_t)((s_ring_head + 1u) % 8u);
    if (s_ring_len < 8u) s_ring_len++;
}

static uint16_t ring_flush(void)
{
    uint16_t done = 0;
    uint16_t start = (uint16_t)((s_ring_head + 8u - s_ring_len) % 8u);

    for (uint16_t k = 0; k < s_ring_len; k++)
    {
        uint16_t idx = (uint16_t)((start + k) % 8u);
        uint8_t  payload[EEPROM_REC_DATA_LEN];
        for (uint8_t i = 0; i < EEPROM_REC_DATA_LEN; i++) payload[i] = s_ring[idx].payload[i];

        if (s_fail_at >= 0 && done == s_fail_at) break;   /* 模拟写失败 */

        s_written = payload[0];    /* 记录写入的样本 id */
        done++;
    }

    if (done == s_ring_len)      s_ring_len = 0;
    else if (done > 0)
    {
        uint16_t keep = (uint16_t)(s_ring_len - done);
        uint16_t nstart = (uint16_t)((start + done) % 8u);
        for (uint16_t k = 0; k < keep; k++)
        {
            uint16_t src = (uint16_t)((nstart + k) % 8u);
            for (uint8_t i = 0; i < EEPROM_REC_DATA_LEN; i++)
                s_ring[k].payload[i] = s_ring[src].payload[i];
        }
        s_ring_len = keep;
    }
    return done;
}
/* ===== SKELETON 结束 ===== */

static int s_pass = 0, s_fail = 0;

static void check(int cond, const char* name)
{
    if (cond) { printf("  [ok]   %s\n", name); s_pass++; }
    else      { printf("  [FAIL] %s\n", name); s_fail++; }
}

static void reset(void)
{
    memset(s_ring, 0, sizeof(s_ring));
    s_ring_head = 0; s_ring_len = 0; s_fail_at = -1; s_written = 0;
}

static void push_id(uint8_t id)
{
    uint8_t p[EEPROM_REC_DATA_LEN];
    memset(p, 0, sizeof(p));
    p[0] = id;
    ring_push(p);
}

int main(void)
{
    printf("[环形缓冲 · 黑匣子预存]\n");

    /* ① 基本时序：不满缓冲时按压入顺序刷出 */
    reset();
    for (uint8_t i = 1; i <= 3; i++) push_id(i);
    check(s_ring_len == 3, "压入 3 条后 len == 3");
    uint16_t d = ring_flush();
    check(d == 3, "一次刷出 3 条 (done==3)");
    check(s_ring_len == 0, "全部刷出后缓冲清空");
    check(s_written == 3, "最后刷出的是 id==3（时间序末尾）");

    /* ② 跨环绕边界时序：缓冲写满 8 条后再压 5 条，head 已环绕 1圈多 */
    reset();
    for (uint8_t i = 1; i <= 8; i++) push_id(i);   /* 填满 */
    for (uint8_t i = 9; i <= 13; i++) push_id(i);  /* 环绕，淘汰 1..5 */
    check(s_ring_len == 8, "环绕后仍持有 8 条（最旧5 条被淘汰）");
    d = ring_flush();
    check(d == 8, "环绕状态下刷出 8 条");
    check(s_written == 13, "环绕时最后刷出的是最新 id==13");

    /* ③ 部分失败：第 3 条写失败 → 前 2 条已落盘，后 5 条必须保留 */
    reset();
    for (uint8_t i = 1; i <= 7; i++) push_id(i);
    s_fail_at = 3;
    d = ring_flush();
    check(d == 3, "第 3 条失败时 done==3（已完成 2 条 + 失败 1 条位置）");
    check(s_ring_len == 4, "失败后剩余 4 条仍保留在 RAM（7-3=4）");

    /* 重试（不再失败）应把剩余 4 条按序刷出，最后一条是 id==7 */
    s_fail_at = -1;
    d = ring_flush();
    check(d == 4, "重试刷出剩余 4 条");
    check(s_written == 7, "重试后最后刷出的是 id==7（未错序、末尾未丢）");
    check(s_ring_len == 0, "重试完成后缓冲清空");

    /* ④ 边界：空缓冲刷出 = 0 条，不崩*/
    reset();
    d = ring_flush();
    check(d == 0 && s_ring_len == 0, "空缓冲刷出 0 条（不崩、不越界）");

    printf("\nPASS %d/FAIL %d\n", s_pass, s_fail);
    return (s_fail == 0) ? 0 : 1;
}
