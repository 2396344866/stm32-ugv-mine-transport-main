/**
  * @file    scheduler.c
  * @brief   时间触发调度内核：20ms 确定性控制节拍（TIM3 更新中断）
  *
  *  设计要点：
  *   - 控制节拍由独立硬件定时器 TIM3 产生（72MHz/7200/200 = 20ms），
  *     与 FreeRTOS 1kHz SysTick 解耦，保证控制环的确定性。
  *   - 中断仅释放二值信号量，实际 PID 运算在控制任务上下文执行，
  *     符合"中断底半部"原则，缩短关中断时间。
  */
#include "scheduler.h"
#include "stm32f10x.h"
#include "misc.h"
#include "board_config.h"

SemaphoreHandle_t xControlSemaphore = NULL;

void Scheduler_Init(void)
{
    xControlSemaphore = xSemaphoreCreateBinary();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler     = 7199;   /* 72MHz / 7200 = 10kHz */
    TIM_TimeBaseStructure.TIM_Period        = 199;    /* 10kHz * 200 = 20ms   */
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    /* 优先级 12 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(11)，可安全调用 FreeRTOS API */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM3, ENABLE);
}

void Scheduler_ControlTickISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xControlSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
