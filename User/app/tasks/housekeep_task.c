/**
  * @file    housekeep_task.c
  * @brief   系统守护任务：独立看门狗喂狗 + 任务健康巡检 + 复位溯源
  *
  *  高稳定性设计核心：
  *    - IWDG 独立看门狗由独立任务周期性喂狗；若任何高优先级任务
  *      长期卡死导致本任务无法及时喂狗，硬件将触发系统复位；
  *    - Health_Check 检测各业务任务是否在窗口内"汇报存活"，卡死即记错；
  *    - 启动期检测 IWDG/软件复位标志，记录 ERR_WATCHDOG_RESET 以便溯源。
  */
#include "tasks.h"
#include "bsp_watchdog.h"
#include "health.h"
#include "error.h"

#define HOUSEKEEP_PERIOD_MS   (200)   /* 喂狗周期，须远小于 IWDG 超时 */
#define IWDG_TIMEOUT_MS       (1000)  /* 看门狗超时(1s)：喂狗周期 200ms 留足余量 */

void Housekeep_Task(void* pvParameters)
{
    (void)pvParameters;

    Health_Register(TASK_ID_HOUSEKEEP);

    /* 复位溯源：独立看门狗复位标志 */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
    {
        Error_Record(ERR_WATCHDOG_RESET);
        RCC_ClearFlag();
    }

    BSP_IWDG_Init(IWDG_TIMEOUT_MS);

    for (;;)
    {
        Health_Report(TASK_ID_HOUSEKEEP);

        /* 任务存活巡检（检测业务任务卡死） */
        Health_Check();

        /* 刷新全局计数 */
        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                g_state.uptime_sec = Health_UptimeSec();
                g_state.err_total  = Error_CountTotal();
                g_state.last_err   = Error_Last();
                xSemaphoreGive(xSystemStateMutex);
            }
        }

        /* 喂独立看门狗（必须在超时前完成） */
        BSP_IWDG_Feed();

        vTaskDelay(pdMS_TO_TICKS(HOUSEKEEP_PERIOD_MS));
    }
}
