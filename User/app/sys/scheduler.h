#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "FreeRTOS.h"
#include "semphr.h"

/* 控制节拍二值信号量：由 TIM3 20ms 中断释放，控制任务据此同步 */
extern SemaphoreHandle_t xControlSemaphore;

/**
  * @brief  初始化 20ms 时间触发控制节拍（TIM3 硬件定时器）
  *         并创建控制节拍信号量。需在使用前调用一次。
  */
void Scheduler_Init(void);

/**
  * @brief  在 TIM3 更新中断中调用：释放控制节拍信号量。
  *         中断优先级需 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY。
  */
void Scheduler_ControlTickISR(void);

#endif /* SCHEDULER_H */
