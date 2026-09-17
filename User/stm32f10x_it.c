/**
  * @file    stm32f10x_it.c
  * @brief   中断服务例程：FreeRTOS 内核异常 + 20ms 控制节拍(TIM3)
 *          + UART 接收 + 车身 CAN 接收(FIFO0)
  */
#include "stm32f10x_it.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board_config.h"
#include "scheduler.h"
#include "bsp_usart.h"
#include "bsp_can.h"

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

/* ----------------------- 车身 CAN 接收中断（FIFO0 挂号） ----------------------- */
/* F103 中该向量与 USB 低优先级中断共用，工程未启用 USB，独占使用。
   ISR 内只把帧搬进接收队列，协议解析与状态迁移在 BCM 任务上下文完成。 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    BSP_CAN_RxFifo0ISR();
}
