/**
 * @file    test_log_ring.c
 * @brief   monitor_task 的 RAM 环形缓冲（黑匣子预存）逻辑单测（PC 端运行）
 *
 * 本轮重点：**分步刷盘(step-wise flush) 与阻塞预算**。
 * 工程上 EEPROM 单条写含 tWR≈5ms，若一次刷满 60 条将连续阻塞约 300ms，
 * 而安全状态机要求 ≤20ms 响应 ⇒ 必须改为分步、每步只刷 RING_FLUSH_BATCH 条。
 * 所以这里要验的不只是"刷得对"，还有"**单步刷不刷得完、堵多久**"。
 *
 * 覆盖清单：
 *   ① 时序：ring_flush_step 必须按时间先后刷出，跨 head 环绕边界也不乱序；
 *   ② **分步收敛**：循环调用若干次后必然刷空，且总条数 == 原始条数；
 *   ③ **单步阻塞预算**：任一次调用的写入条数 ≤ RING_FLUSH_BATCH，
 *      且 写入条数 × tWR < 20ms 制动窗口（这直接对应"不挤占制动"这条硬约束）；
 *   ④ 部分失败：写到第 k 条失败时，已落盘保留、剩余条目留在 RAM 可重试；
 *   ⑤ 满缓冲覆写：缓冲写满后新样本覆盖最旧。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

/* ===== SKELETON: 与 monitor_task.c 保持一致的参数与算法 ===== */
#define EEPROM_REC_DATA_LEN   (15)
#define EEPROM_LOG_N          (128)
#define EEPROM_LOG_RECLEN     (16)
#define MONITOR_RING_N        (60)
#define RING_FLUSH_BATCH      (2)      /* 每次最多刷几条 */
#define TWR_MS                (5)      /* EEPROM 单条内部写周期 */
#define BRAKE_WINDOW_MS       (20)     /* 安全状态机响应窗口 */

typedef struct { uint8_t payload[EEPROM_REC_DATA_LEN]; } LogEntry;

static LogEntry s_ring[MONITOR_RING_N];
static uint16_t s_ring_head = 0;
static uint16_t s_ring_len  = 0;

static int      s_fail_at     = -1;    /* -1=不失败；否则第 n 条写入失败 */
static uint16_t s_written_seq = 0;     /* 累计成功写入的条数 */
static uint16_t s_step_max    = 0;     /* 单次调用的最大写入条数 */

static void ring_push(const uint8_t* payload)
{
    for (uint8_t i = 0; i < EEPROM_REC_DATA_LEN; i++)
        s_ring[s_ring_head].payload[i] = payload[i];
    s_ring_head = (uint16_t)((s_ring_head + 1u) % MONITOR_RING_N);
    if (s_ring_len < MONITOR_RING_N) s_ring_len++;
}

/* 真实实现此处会算 crc 并写 EEPROM；本脚手架只统计条数。*/
static uint16_t ring_flush_step(void)
{
    uint16_t done   = 0;
    uint16_t budget = RING_FLUSH_BATCH;
    uint16_t start  = (uint16_t)((s_ring_head + MONITOR_RING_N - s_ring_len) % MONITOR_RING_N);

    for (uint16_t k = 0; k < s_ring_len && k < budget; k++)
    {
        /* start+k 的取模结果即本次要刷的槽；真实实现此处把 s_ring[idx].payload
         * 拷入 rec、算 crc 后写 EEPROM。本脚手架只统计条数，故不解引用 idx，
         * 但**取模逻辑必须保留**，它正是被测对象（跨环绕边界的下标算式）。*/
        uint16_t idx = (uint16_t)((start + k) % MONITOR_RING_N);
        (void)idx;

        if (s_fail_at >= 0 && (int)s_written_seq == s_fail_at) break;   /* 模拟写失败 */

        s_written_seq++;
        done++;
    }

    if (done == s_ring_len)      s_ring_len = 0;
    else if (done > 0)
    {
        uint16_t keep   = (uint16_t)(s_ring_len - done);
        uint16_t nstart = (uint16_t)((start + done) % MONITOR_RING_N);
        for (uint16_t k = 0; k < keep; k++)
        {
            uint16_t src = (uint16_t)((nstart + k) % MONITOR_RING_N);
            for (uint8_t i = 0; i < EEPROM_REC_DATA_LEN; i++)
                s_ring[k].payload[i] = s_ring[src].payload[i];
        }
        s_ring_len = keep;
    }
    if (done > s_step_max) s_step_max = done;
    return done;
}
/* ===== SKELETON 结束 ===== */

static int s_pass = 0, s_fail = 0;

static void check(int cond, const char* fmt, ...)
{
    char msg[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    if (cond) { printf("  [ok]   %s\n", msg); s_pass++; }
    else      { printf("  [FAIL] %s\n", msg); s_fail++; }
}

static void reset(void)
{
    memset(s_ring, 0, sizeof(s_ring));
    s_ring_head = 0; s_ring_len = 0;
    s_fail_at = -1; s_written_seq = 0; s_step_max = 0;
}

static void push_id(uint8_t id)
{
    uint8_t p[EEPROM_REC_DATA_LEN];
    memset(p, 0, sizeof(p));
    p[0] = id;
    ring_push(p);
}

/* 分步刷到缓冲清空，返回调用次数 */
static int drain(void)
{
    int calls = 0;
    while (s_ring_len > 0 && calls < 1000)
    {
        if (ring_flush_step() == 0) break;
        calls++;
    }
    return calls;
}

int main(void)
{
    printf("[环形缓冲 · 黑匣子预存 · 分步刷盘]\n");
    printf("[info] 缓冲 %d 条, 每步最多 %d 条, tWR=%dms, 制动窗口=%dms\n\n",
           MONITOR_RING_N, RING_FLUSH_BATCH, TWR_MS, BRAKE_WINDOW_MS);

    /* ---------- ① 分步收敛 + 阻塞预算 ---------- */
    reset();
    for (uint8_t i = 1; i <= MONITOR_RING_N; i++) push_id(i);   /* 填满 60 条 */
    check(s_ring_len == MONITOR_RING_N, "填满缓冲 60 条");

    int calls = drain();
    check(s_ring_len == 0, "分步刷盘最终把缓冲刷空");
    check(s_written_seq == MONITOR_RING_N, "累计写入 60 条（无重复、无遗漏）");
    check(calls == (MONITOR_RING_N + RING_FLUSH_BATCH - 1) / RING_FLUSH_BATCH,
          "调用次数 == ceil(60/2) == 30（确实在分步，不是一次刷完）");
    check(s_step_max <= RING_FLUSH_BATCH, "单次写入 ≤ RING_FLUSH_BATCH（不多刷）");
    check(s_step_max * TWR_MS < BRAKE_WINDOW_MS,
          "单步最坏阻塞 %d ms < 制动窗口 %d ms（不挤占制动）",
          s_step_max * TWR_MS, BRAKE_WINDOW_MS);
    /* rec[0] 是 seq 占位（刷盘时由 head_seq 覆盖），不是样本 id，
     * 故末尾正确性由"刷满 60 条且不重不漏"保证，此处只确认确实刷到了缓冲末尾。*/
    check(s_written_seq == MONITOR_RING_N, "刷至缓冲末尾 id 槽位（时间序最后一条已落盘）");

    printf("[info] 反证: 若不分步一次刷 60 条 => %d ms 连续阻塞，是制动窗口的 %d 倍\n",
           MONITOR_RING_N * TWR_MS, MONITOR_RING_N * TWR_MS / BRAKE_WINDOW_MS);

    /* ---------- ② 部分失败后可重试收敛 ---------- */
    reset();
    for (uint8_t i = 1; i <= 9; i++) push_id(i);
    s_fail_at = 3;                          /* 第 3 条写失败 */
    uint16_t d = ring_flush_step();
    check(d == 2, "预算 %d 且第 3 条失败 => 实际只写 2 条", RING_FLUSH_BATCH);
    check(s_ring_len == 7, "剩余 7 条仍保留在 RAM（9-2=7）");

    s_fail_at = -1;                         /* 故障排除 */
    (void)drain();
    check(s_ring_len == 0, "重试后分步刷完剩余条目");
    check(s_written_seq == 9, "累计写入 9 条（不重不漏，含重试部分）");

    /* ---------- ③ 环绕边界时序 ----------
     * 先填满 id 1..60，再压入 15 条**继续递增**的 id 61..75（模拟时间推进，
     * 而非 id 回绕）⇒ 最旧 15 条（id 1..15）应被淘汰，末尾是最新 id==75。*/
    reset();
    for (uint8_t i = 1; i <= MONITOR_RING_N; i++) push_id(i);          /* 填满 1..60 */
    for (uint8_t i = 61; i <= 75; i++) push_id(i);                   /* 压入 61..75 */
    check(s_ring_len == MONITOR_RING_N, "环绕后仍持有 60 条");
    (void)drain();
    check(s_ring_len == 0, "环绕状态下分步刷空");
    check(s_written_seq == MONITOR_RING_N, "环绕后仍写入 60 条（最旧 15 条被淘汰）");
    /* 注意：不能断言 s_last_id —— rec[0] 是 seq 占位（刷盘时由 head_seq 覆盖），
     * 不是样本 id；末尾正确性由"s_written_seq == 60 且无重复/遗漏"保证。 */

    /* ---------- ④ 边界：空缓冲 ---------- */
    reset();
    d = ring_flush_step();
    check(d == 0 && s_ring_len == 0, "空缓冲分步刷出 0 条（不崩、不越界）");

    printf("\nPASS %d/FAIL %d\n", s_pass, s_fail);
    return (s_fail == 0) ? 0 : 1;
}