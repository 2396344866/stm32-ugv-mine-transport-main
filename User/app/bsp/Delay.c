/**
  * @file    Delay.c
  * @brief   基于 Cortex-M3 DWT _cycle 计数器的延时（不占用 SysTick，
  *          与 FreeRTOS 的 SysTick 节拍互不干扰）。
  */
#include "stm32f10x.h"
#include "core_cm3.h"
#include "dwt.h"
#include "Delay.h"

static uint32_t g_fac_us = 0;

void Delay_Init(void)
{
    /* 使能 DWT 跟踪单元 */
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    g_fac_us = SystemCoreClock / 1000000U;
}

void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * g_fac_us;
    while ((DWT->CYCCNT - start) < cycles);
}

void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}

void Delay_s(uint32_t s)
{
    while (s--)
    {
        Delay_ms(1000);
    }
}
