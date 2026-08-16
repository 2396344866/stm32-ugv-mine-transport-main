/**
  * @file    bsp_rtc.c
  * @brief   RTC（LSI 内部低速时钟）+ STOP 低功耗唤醒
  */
#include "bsp_rtc.h"
#include "stm32f10x.h"
#include <stdio.h>

int BSP_RTC_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
    {
        RCC_LSICmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

        RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
        RCC_RTCCLKCmd(ENABLE);
        RTC_WaitForSynchro();
        RTC_WaitForLastTask();

        /* LSI ~40kHz -> 1Hz 计数（分频 40000-1） */
        RTC_SetPrescaler(39999);
        RTC_WaitForLastTask();

        RTC_SetCounter(0);
        RTC_WaitForLastTask();

        BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
    }
    else
    {
        RTC_WaitForSynchro();
    }
    return 0;
}

void BSP_RTC_GetTime(uint8_t* hour, uint8_t* min, uint8_t* sec)
{
    uint32_t t = RTC_GetCounter();
    *sec  = t % 60;
    *min  = (t / 60) % 60;
    *hour = (t / 3600) % 24;
}

int BSP_RTC_SetWakeupSeconds(uint32_t sec)
{
    RTC_WaitForLastTask();
    RTC_SetAlarm(RTC_GetCounter() + sec);
    RTC_WaitForLastTask();

    /* 使能 RTC 闹钟 -> EXTI17 唤醒（从 STOP 唤醒无需 NVIC） */
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_ClearITPendingBit(EXTI_Line17);
    EXTI_InitStructure.EXTI_Line    = EXTI_Line17;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    return 0;
}

void BSP_EnterStopUntilWakeup(void)
{
    /* 清除唤醒标志，进入 STOP（低功耗），由 RTC 闹钟事件唤醒 */
    PWR_ClearFlag(PWR_FLAG_WU);
    RTC_ClearFlag(RTC_FLAG_ALR);
    EXTI_ClearITPendingBit(EXTI_Line17);

    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

    /* 唤醒后：HSE/PLL 已停，重新初始化系统时钟 */
    SystemInit();
}
