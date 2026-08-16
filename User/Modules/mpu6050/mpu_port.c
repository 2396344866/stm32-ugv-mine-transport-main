/**
  * @file    mpu_port.c
  * @brief   Motion Driver(EMPL_TARGET_STM32F1) 平台适配：毫秒时基
  *
  *  inv_mpu 在 STM32 平台以宏 `get_ms -> get_tick_count` 获取毫秒时基。
  *  以 DWT 周期计数器经 SystemCoreClock 折算，无需额外硬件定时器，
  *  在调度器启动前后均可使用（DMP 初始化早于 RTOS 启动）。
  */
#include "stm32f10x.h"
#include "core_cm3.h"
#include "dwt.h"

unsigned long get_tick_count(unsigned long *count)
{
    unsigned long ms = 0;
    if (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)
    {
        /* DWT 已使能（Delay_Init 开启） */
        ms = DWT->CYCCNT / (SystemCoreClock / 1000UL);
    }
    if (count) *count = ms;
    return 0;
}
