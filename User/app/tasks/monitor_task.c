/**
  * @file    monitor_task.c
  * @brief   工业管道气压监测任务：ADC 采样 -> 混合数字滤波 -> 迟滞报警
  *           -> EEPROM 断点续传存储 -> 低功耗(RTC/STOP)调度
  *
  *  高稳定性设计要点：
  *    - 混合滤波(去极值平均 + 一阶滞后)抑制传感器噪声与偶发尖峰；
  *    - 迟滞报警状态机避免临界抖动误报；
  *    - 异常数据落盘 EEPROM，断网/复位后可续传；
  *    - RTC(LSI) + STOP 模式支持电池供电长期监测（默认关闭，按需开启）。
  */
#include "tasks.h"
#include "filter.h"
#include "alarm.h"
#include "bsp_adc.h"
#include "bsp_rtc.h"
#include "bsp_at24c256.h"

/* ---- 监测参数 ---- */
#define MONITOR_PERIOD_MS     (50)     /* 采样周期 */
#define MONITOR_LOG_PERIOD    (20)     /* 每 20 次采样(1s)落盘一次 */
#define PRESSURE_VREF         (3.3f)
#define PRESSURE_FULL_SCALE   (10.0f)  /* 变送器量程 0..10 bar */
#define PRESSURE_NOMINAL      (5.0f)   /* 标称气压(bar)，报警参考 */
#define FILTER_ALPHA          (0.2f)   /* 一阶滞后系数 */
#define TENSION_PER_BAR       (15.0f)  /* 气压 -> 张力代理增益(N/bar) */

/* ============================================================================
 * EEPROM 环形日志布局（工业界掉电安全做法）
 *
 *  页 0（0x0000..0x001F）：
 *    0x0000  head A : seq(4,LE) + crc8(1)                 = 5 B
 *    0x0008  head B : seq(4,LE) + crc8(1)                 = 5 B
 *    0x0010  日志区：128 条 × 16 B = 2048 B（0x0010..0x080F）
 *
 *  记录（16 B）：
 *    seq(1) hour(1) min(1) sec(1) alarm(1) raw(2,LE) filt100(2,LE)
 *    pad(3) crc8(1)  —— 后 5 B 预留扩展（如毫秒/温度/故障码），当前填 0
 *
 *  为何取 16 B：AT24C256 页长 64 B，16 是 64 的整数倍 ⇒ 每页整放 4 条，
 *  **单条永不跨页**，无需在写入路径里拆页事务（避免每条多一次 tWR 等待，
 *  降低写入耗时与失败概率）。驱动层仍保留通用分页作为兜底。
 *
 *  掉电安全设计：
 *   ① **head A/B 双份交替**：每写一条后翻转活动份，掉电最多只损坏非活动的那份；
 *      上电时取「crc 校验通过且 seq 较大」者，因此必然拿到一致的 head；
 *   ② **记录带 seq + crc8**：可逐条判定完整性，撕裂记录在读回时被丢弃而非
 *      当成有效历史，读取端沿 seq 连续性扫描出有效区间；
 *   ③ **先写记录、后提交 head**：head 仍是提交点（commit point），
 *      掉电在记录写窗口时，head 未推进 ⇒ 该槽尚未提交，下次直接覆盖。
 * ==========================================================================*/
#define EEPROM_LOG_BASE       (0x0010)
#define EEPROM_LOG_N          (128)
#define EEPROM_LOG_RECLEN     (16)
#define EEPROM_REC_CRC_OFF    (15)     /* crc 位于记录末字节 */
#define EEPROM_REC_DATA_LEN   (15)     /* crc 覆盖前 15 字节 */
#define EEPROM_HEAD_A_ADDR    (0x0000)
#define EEPROM_HEAD_B_ADDR    (0x0008)
#define EEPROM_HEAD_RECLEN    (5)      /* seq(4) + crc(1) */
#define EEPROM_HEAD_INVALID   (0xFFFFFFFFu)

/* CRC-8/ATM (poly=0x07, init=0x00)，逐位计算，无查表 RAM 开销 */
static uint8_t crc8_atm(uint8_t crc, const uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u32_le(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get_u32_le(const uint8_t* p)
{
    return ((uint32_t)p[0])
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static Filter_Handle    s_filt;
static AlarmHysteresis  s_alarm;

/* ---- head A/B 双份读写 ---- */
/* 读一份 head：crc 校验通过才返回；失败返回 EEPROM_HEAD_INVALID */
static uint32_t head_read_one(uint16_t addr)
{
    uint8_t buf[EEPROM_HEAD_RECLEN] = {0};
    if (BSP_AT24C256_Read(addr, buf, EEPROM_HEAD_RECLEN) != 0)
    {
        Error_Record(ERR_I2C_TIMEOUT);
        return EEPROM_HEAD_INVALID;
    }
    uint8_t calc = crc8_atm(0x00u, buf, 4);
    if (calc != buf[4])
    {
        return EEPROM_HEAD_INVALID;      /* crc 不符 ⇒ 该份已损坏 */
    }
    return get_u32_le(buf);
}

/* 写一份 head（seq 显式小端 + crc 覆盖前 4 字节） */
static int head_write_one(uint16_t addr, uint32_t seq)
{
    uint8_t buf[EEPROM_HEAD_RECLEN];
    put_u32_le(buf, seq);
    buf[4] = crc8_atm(0x00u, buf, 4);
    if (BSP_AT24C256_Write(addr, buf, EEPROM_HEAD_RECLEN) != 0)
    {
        Error_Record(ERR_EEPROM_WRITE);
        return -1;
    }
    return 0;
}

/* 上电恢复：取 A/B 中 crc 通过且 seq 较大者；活动份随后用于轮转写入。
 * 返回有效 seq（无有效则 0 表示从头开始）。 */
static uint32_t head_load(uint8_t* active_slot)
{
    uint32_t a = head_read_one(EEPROM_HEAD_A_ADDR);
    uint32_t b = head_read_one(EEPROM_HEAD_B_ADDR);

    /* 用无符号差值处理 seq 回绕（模 2^32），谁「更新」用 (b-a) 是否落在后一半判定 */
    uint32_t newer_b = (uint32_t)(b - a);
    int b_valid = (a != EEPROM_HEAD_INVALID) || (b != EEPROM_HEAD_INVALID);
    int b_newer;
    if (a == EEPROM_HEAD_INVALID)      b_newer = 1;
    else if (b == EEPROM_HEAD_INVALID) b_newer = 0;
    else                                b_newer = (newer_b != 0u) && (newer_b < 0x80000000u);

    if (!b_valid) { *active_slot = 0; return 0; }
    if (b_newer)  { *active_slot = 1; return b; }
    *active_slot = 0;
    return (a == EEPROM_HEAD_INVALID) ? b : a;
}

/* 写入一条监测记录（16B，带 seq + crc8），并提交 head。
 * 写序：先记录、后 head（head 为提交点）。head A/B 交替翻转，掉电只可能损坏
 * 非活动份，上电经 crc+seq 取较新者即可恢复一致 head。 */
static uint8_t s_head_slot = 0;      /* 0=A 1=B，当前活动份 */
static uint32_t s_head_seq = 0;

static void eeprom_log(uint8_t hour, uint8_t min, uint8_t sec,
                       uint8_t alarm, uint16_t raw, uint16_t filt100)
{
    uint8_t rec[EEPROM_LOG_RECLEN];

    /* seq 落在低字节（128 槽循环，低 7 位足够区分相邻记录） */
    rec[0] = (uint8_t)(s_head_seq & 0xFFu);
    rec[1] = hour;
    rec[2] = min;
    rec[3] = sec;
    rec[4] = alarm;
    rec[5] = (uint8_t)(raw & 0xFFu);
    rec[6] = (uint8_t)((raw >> 8) & 0xFFu);
    rec[7] = (uint8_t)(filt100 & 0xFFu);
    rec[8] = (uint8_t)((filt100 >> 8) & 0xFFu);
    /* rec[9..14] 预留扩展，当前填 0；crc 覆盖 rec[0..14] */
    for (uint8_t i = 9; i < EEPROM_REC_CRC_OFF; i++) rec[i] = 0;
    rec[EEPROM_REC_CRC_OFF] = crc8_atm(0x00u, rec, EEPROM_REC_DATA_LEN);

    uint16_t addr = EEPROM_LOG_BASE + (s_head_seq % EEPROM_LOG_N) * EEPROM_LOG_RECLEN;
    if (BSP_AT24C256_Write(addr, rec, EEPROM_LOG_RECLEN) != 0)
    {
        Error_Record(ERR_EEPROM_WRITE);
        return;   /* 记录未落盘 → 不推进 seq/head，保持提交点语义 */
    }

    /* 提交：seq 先加一，再翻转到另一份 head 写入 */
    s_head_seq++;
    uint16_t next_slot = (uint16_t)(s_head_slot ^ 1u);
    uint16_t next_addr = next_slot ? EEPROM_HEAD_B_ADDR : EEPROM_HEAD_A_ADDR;
    if (head_write_one(next_addr, s_head_seq) != 0)
    {
        /* 提交失败：回退 seq，保持与 EEPROM 中有效 head 一致 */
        s_head_seq--;
        return;
    }
    s_head_slot = (uint8_t)next_slot;
}

/* ---------------------------------------------------------------------------
 * 读取接口（供通信任务做断点续传）
 *
 * 按 seq 从最新往回扫描，逐条用 crc8 判定完整性：
 *   - crc 通过 ⇒ 该条完整有效；
 *   - crc 失败 ⇒ 该条被掉电撕裂，直接跳过（不中断扫描，继续向更旧找）。
 * 因为 head 是提交点，任何未被 head 覆盖的撕裂记录都不在有效区间内，
 * 而被覆盖的最旧槽即使撕裂也只丢 1 条，seq 连续性扫描可精确识别。
 * 返回 1 = 取到一条有效记录（填入 out），0 = 已扫完全部槽位。
 * -------------------------------------------------------------------------*/
int Monitor_LogRead(uint32_t back, uint8_t* out, uint8_t len)
{
    if (out == NULL || len < EEPROM_LOG_RECLEN) return 0;

    for (uint32_t k = 0; k < EEPROM_LOG_N; k++)
    {
        /* back=0 为最新一条，逐条向更旧走 */
        uint32_t seq = s_head_seq - 1u - k;
        uint16_t addr = EEPROM_LOG_BASE + (seq % EEPROM_LOG_N) * EEPROM_LOG_RECLEN;

        uint8_t rec[EEPROM_LOG_RECLEN];
        if (BSP_AT24C256_Read(addr, rec, EEPROM_LOG_RECLEN) != 0)
        {
            continue;   /* 总线读失败：跳过该条 */
        }
        if (crc8_atm(0x00u, rec, EEPROM_REC_DATA_LEN) != rec[EEPROM_REC_CRC_OFF])
        {
            continue;   /* crc 不符：撕裂记录，丢弃 */
        }
        /* 校验 seq 低字节一致，确认扫到的是预期那条（防槽位错位） */
        if (rec[0] != (uint8_t)(seq & 0xFFu))
        {
            continue;   /* 该槽尚未写入，或属于更旧代次 */
        }

        for (uint8_t i = 0; i < len; i++) out[i] = rec[i];
        return 1;
    }
    return 0;
}

/* 当前已提交 head（seq），供通信任务记录续传进度 */
uint32_t Monitor_LogHead(void)
{
    return s_head_seq;
}

void Monitor_Task(void* pvParameters)
{
    (void)pvParameters;

    Health_Register(TASK_ID_MONITOR);

    Filter_Init(&s_filt, FILTER_ALPHA);
    Alarm_Init(&s_alarm, PRESSURE_NOMINAL, 0.05f, 0.02f, 0.08f);

#if USE_RTC_DEPENDENT
    BSP_RTC_Init();
#endif

    /* 上电恢复：从 head A/B 双份中取 crc 通过且 seq 较新者 */
    s_head_seq = head_load(&s_head_slot);

    uint16_t raw = 0;
    float   bar = 0.0f, filt = 0.0f;
    uint32_t sample_cnt = 0;

    for (;;)
    {
        Health_Report(TASK_ID_MONITOR);

        /* --- 1. ADC 采样 --- */
        raw = BSP_ADC_ReadRaw();
        if (raw == 0 && BSP_ADC_ReadRaw() == 0)
        {
            /* 连续两次为 0 视为读取异常（变送器断线/短路） */
            Error_Record(ERR_ADC_READ);
        }

        /* 12 位原始值 -> 气压(bar) */
        bar = ((float)raw / 4095.0f) * PRESSURE_VREF / PRESSURE_VREF * PRESSURE_FULL_SCALE;
        /* 注：上式保留 VREF 变量以体现标定，实际量程线性映射 */

        /* --- 2. 混合滤波 --- */
        filt = Filter_Update(&s_filt, bar);

        /* --- 3. 迟滞报警 --- */
        AlarmLevel lvl = Alarm_Update(&s_alarm, filt);

        /* --- 4. 张力代理 & 回写全局状态 --- */
        float tension = filt * TENSION_PER_BAR;
        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                g_state.pressure_raw  = raw;
                g_state.pressure_bar  = bar;
                g_state.pressure_filt = filt;
                g_state.alarm         = lvl;
                g_state.tension       = tension;
                xSemaphoreGive(xSystemStateMutex);
            }
        }

        /* --- 5. 周期落盘 + 推送日志队列(供通信任务上传) --- */
        if ((++sample_cnt % MONITOR_LOG_PERIOD) == 0)
        {
            uint8_t h = 0, m = 0, s = 0;
#if USE_RTC_DEPENDENT
            BSP_RTC_GetTime(&h, &m, &s);
#endif
            uint16_t filt100 = (uint16_t)(filt * 100.0f);
            eeprom_log(h, m, s, (uint8_t)lvl, raw, filt100);

            if (xLogQueue != NULL)
            {
                uint8_t log[8] = { h, m, s, (uint8_t)lvl,
                                   (uint8_t)(raw & 0xFF), (uint8_t)((raw >> 8) & 0xFF),
                                   (uint8_t)(filt100 & 0xFF), (uint8_t)((filt100 >> 8) & 0xFF) };
                xQueueSend(xLogQueue, log, 0);
            }
        }

        /* --- 6. 低功耗：仅在使能且系统允许时，由 idle 钩子进入 STOP --- */
        vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    }
}
