/**
  * @file    health.c
  * @brief   健康管理（任务看门狗）：检测任务卡死、累计运行时长
  */
#include "health.h"
#include "error.h"
#include "FreeRTOS.h"
#include "task.h"       /* xTaskGetTickCount */

static uint8_t  s_registered[HEALTH_MAX_TASKS];
static uint32_t s_last[HEALTH_MAX_TASKS];
static uint8_t  s_alive[HEALTH_MAX_TASKS];
static uint32_t s_uptime_base;

void Health_Init(void)
{
    for (uint8_t i = 0; i < HEALTH_MAX_TASKS; i++)
    {
        s_registered[i] = 0; s_last[i] = 0; s_alive[i] = 0;
    }
    s_uptime_base = xTaskGetTickCount();
}

void Health_Register(uint8_t id)
{
    if (id < HEALTH_MAX_TASKS) s_registered[id] = 1;
}

void Health_Report(uint8_t id)
{
    if (id < HEALTH_MAX_TASKS)
    {
        s_last[id] = xTaskGetTickCount();
        s_alive[id] = 1;
    }
}

void Health_Check(void)
{
    uint32_t now = xTaskGetTickCount();
    for (uint8_t i = 0; i < HEALTH_MAX_TASKS; i++)
    {
        if (!s_registered[i]) continue;
        if ((now - s_last[i]) > HEALTH_WINDOW_TICKS)
        {
            if (s_alive[i]) Error_Record(ERR_UNKNOWN);
            s_alive[i] = 0;   /* 标记卡死 */
        }
    }
}

uint8_t Health_IsAlive(uint8_t id)
{
    return (id < HEALTH_MAX_TASKS) ? s_alive[id] : 0;
}

uint32_t Health_UptimeSec(void)
{
    return (xTaskGetTickCount() - s_uptime_base) / configTICK_RATE_HZ;
}
