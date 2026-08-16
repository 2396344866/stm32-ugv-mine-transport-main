/**
  * @file    stm32f10x_it.c
  * @brief   中断服务例程：FreeRTOS 内核异常 + 20ms 控制节拍(TIM3) + UART 接收
  */
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board_config.h"
#include "scheduler.h"
#include "bsp_usart.h"

/* FreeRTOS ARM_CM3 移植层提供的内核异常入口（定义于 port.c，无公共头原型） */
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

/* ----------------------- Cortex-M3 内核异常（FreeRTOS 接管） ----------------------- */
void NMI_Handler(void)        { for (;;); }
void HardFault_Handler(void)  { for (;;); }
void MemManage_Handler(void)  { for (;;); }
void BusFault_Handler(void)   { for (;;); }
void UsageFault_Handler(void) { for (;;); }

void SVC_Handler(void)
{
    vPortSVCHandler();
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
    xPortPendSVHandler();
}

void SysTick_Handler(void)
{
    xPortSysTickHandler();
}

/* ----------------------- 20ms 确定性控制节拍（TIM3 更新中断） ----------------------- */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        Scheduler_ControlTickISR();   /* 释放控制节拍二值信号量 */
    }
}

/* ----------------------- UART 接收中断（ESP8266 / 调试 / UWB） ----------------------- */
void USART1_IRQHandler(void)
{
    BSP_USART_IRQHandler(USART1);
}

void USART2_IRQHandler(void)
{
    BSP_USART_IRQHandler(USART2);
}

void USART3_IRQHandler(void)
{
    BSP_USART_IRQHandler(USART3);
}
