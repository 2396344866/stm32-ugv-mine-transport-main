/**
  * @file    main.c
  * @brief   STM32F103RCT6 工业物联网边缘节点 — 固件入口
  *
  *  系统架构：
  *    - 硬件初始化(SystemInit 已由 startup 调用，配 72MHz)
  *    - FreeRTOS 抢占式调度：控制环 / 守护 / 监测 / 定位 / 通信 / HMI
  *    - 20ms 确定性控制节拍(TIM3 -> 二值信号量)驱动双闭环 PID
  *    - 高稳定性：IWDG 看门狗 + 任务存活巡检 + 统一错误记录 + 栈溢出钩子
  */
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board_config.h"
#include "scheduler.h"
#include "health.h"
#include "error.h"
#include "tasks.h"
#include "Delay.h"
#include "bsp_usart.h"
#include "bsp_motor.h"
#include "bsp_adc.h"
#include "bsp_at24c256.h"
#include "bsp_rtc.h"
#include "MPU6050.h"

/* 全局对象（定义于本文件，其余模块经 tasks.h 引用） */
SystemState        g_state;
SemaphoreHandle_t  xSystemStateMutex = NULL;
QueueHandle_t      xLogQueue = NULL;

/* ------------------------------------------------------------------ */
/*  FreeRTOS 钩子                                                       */
/* ------------------------------------------------------------------ */
void vApplicationIdleHook(void)
{
    /* 轻量省电：等待中断。深度 STOP 低功耗见 bsp_rtc.c，
     * 可在电池部署中显式配置 RTC 闹钟后调用 BSP_EnterStopUntilWakeup。 */
    __WFI();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    Error_Record(ERR_UNKNOWN);
    for (;;);
}

void vApplicationMallocFailedHook(void)
{
    Error_Record(ERR_UNKNOWN);
    for (;;);
}

/* ------------------------------------------------------------------ */
/*  硬件初始化                                                          */
/* ------------------------------------------------------------------ */
static void System_Init(void)
{
    /* DWT 延时基准（务必先于任何 Delay_ms 调用） */
    Delay_Init();

    /* 调试串口：printf 重定向目标(USART2, PA2/PA3) */
    BSP_USART_Init(DBG_USART, DBG_USART_BAUD, 0);

    /* 控制子系统：电机 PWM/方向/编码器/状态指示 */
    BSP_Motor_Init();

    /* 监测子系统：气压 ADC(PA4) */
    BSP_ADC_Init();

    /* 本地存储：AT24C256(I2C2，与 MPU6050 共享总线) */
#if USE_AT24C256
    BSP_AT24C256_Init();
#endif

    /* 姿态解算：MPU6050 DMP(I2C2) */
#if USE_MPU6050_DMP
    MPU6050_Init();
    if (MPU6050_GetID() == 0x68)
    {
        MPU6050_DMP_Init();
    }
    else
    {
        Error_Record(ERR_MPU_DMP);
    }
#endif

    /* 低功耗时基：RTC(LSI) */
#if USE_RTC_DEPENDENT
    BSP_RTC_Init();
#endif

    /* 通信(UWB/ESP) 与 HMI 的底层 USART/I2C 由各任务自行 Init，
     * 避免启动期阻塞；此处仅完成公共硬件。 */
}

/* ------------------------------------------------------------------ */
/*  主函数                                                              */
/* ------------------------------------------------------------------ */
int main(void)
{
    System_Init();

    /* 跨任务共享资源 */
    xSystemStateMutex = xSemaphoreCreateMutex();
    xLogQueue         = xQueueCreate(16, 8);   /* 监测记录缓冲(8 字节/条) */

    Health_Init();

    /* 20ms 确定性控制节拍（TIM3 -> xControlSemaphore） */
    Scheduler_Init();

    /* 创建任务（优先级：控制环最高） */
    xTaskCreate(Control_Task,   "CTRL",  TASK_STACK_CONTROL, NULL, TASK_PRIO_CONTROL,   NULL);
    xTaskCreate(Housekeep_Task, "HOUSE", TASK_STACK_DEFAULT, NULL, TASK_PRIO_HOUSEKEEP, NULL);
    xTaskCreate(Monitor_Task,   "MON",   TASK_STACK_DEFAULT, NULL, TASK_PRIO_MONITOR,   NULL);
    xTaskCreate(UWB_Task,       "UWB",   TASK_STACK_DEFAULT, NULL, TASK_PRIO_UWB,       NULL);
    xTaskCreate(Comm_Task,      "COMM",  TASK_STACK_DEFAULT, NULL, TASK_PRIO_COMM,      NULL);
    xTaskCreate(HMI_Task,       "HMI",   TASK_STACK_HMI,     NULL, TASK_PRIO_HMI,       NULL);

    vTaskStartScheduler();

    /* 调度器启动失败才会到达此处 */
    for (;;);
}
