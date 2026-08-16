/**
  * @file    bsp_watchdog.c
  * @brief   独立看门狗 IWDG（LSI 40kHz）
  */
#include "bsp_watchdog.h"

void BSP_IWDG_Init(uint16_t timeout_ms)
{
    /* LSI 40kHz；分频 64 -> 625Hz (1.6ms/tick)；reload = ms/1.6，限幅 <= 4095 */
    uint16_t reload = (uint16_t)((timeout_ms * 1000) / 1600);
    if (reload < 1) reload = 1;
    if (reload > 4095) reload = 4095;

    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(reload);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void BSP_IWDG_Feed(void)
{
    IWDG_ReloadCounter();
}
