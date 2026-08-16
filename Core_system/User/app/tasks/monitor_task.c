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

/* EEPROM 环形日志：4 字节头(写计数器) + 128 条 * 8 字节 = 1028 字节 */
#define EEPROM_LOG_BASE       (0x0010)
#define EEPROM_LOG_N          (128)
#define EEPROM_LOG_RECLEN     (8)
#define EEPROM_HEAD_ADDR      (0x0000)

static Filter_Handle    s_filt;
static AlarmHysteresis  s_alarm;

/* 读 EEPROM 中的写计数器(4 字节小端) */
static uint32_t eeprom_read_head(void)
{
    uint8_t buf[4] = {0};
    BSP_AT24C256_Read(EEPROM_HEAD_ADDR, buf, 4);
    return ((uint32_t)buf[0]) | ((uint32_t)buf[1] << 8)
         | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static void eeprom_write_head(uint32_t head)
{
    uint8_t buf[4] = {
        (uint8_t)(head & 0xFF), (uint8_t)((head >> 8) & 0xFF),
        (uint8_t)((head >> 16) & 0xFF), (uint8_t)((head >> 24) & 0xFF)
    };
    BSP_AT24C256_Write(EEPROM_HEAD_ADDR, buf, 4);
}

/* 写入一条监测记录：time(3) + alarm(1) + raw(2) + filt*100(2) */
static void eeprom_log(uint8_t hour, uint8_t min, uint8_t sec,
                       uint8_t alarm, uint16_t raw, uint16_t filt100)
{
    static uint32_t head = 0xFFFFFFFF;
    if (head == 0xFFFFFFFF) head = eeprom_read_head();

    uint8_t rec[EEPROM_LOG_RECLEN] = {
        hour, min, sec, alarm,
        (uint8_t)(raw & 0xFF), (uint8_t)((raw >> 8) & 0xFF),
        (uint8_t)(filt100 & 0xFF), (uint8_t)((filt100 >> 8) & 0xFF)
    };
    uint16_t addr = EEPROM_LOG_BASE + (head % EEPROM_LOG_N) * EEPROM_LOG_RECLEN;
    if (BSP_AT24C256_Write(addr, rec, EEPROM_LOG_RECLEN) != 0)
    {
        Error_Record(ERR_EEPROM_WRITE);
    }
    head++;
    eeprom_write_head(head);
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
