#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include "stm32f10x.h"

/* 独立看门狗（高稳定性设计）：超时未喂狗则系统复位 */
void BSP_IWDG_Init(uint16_t timeout_ms); /* 典型 500~2000ms */
void BSP_IWDG_Feed(void);

#endif /* BSP_WATCHDOG_H */
