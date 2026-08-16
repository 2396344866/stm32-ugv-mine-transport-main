#ifndef HEALTH_H
#define HEALTH_H

#include "stm32f10x.h"
#include "FreeRTOS.h"

#define HEALTH_MAX_TASKS   8
#define HEALTH_WINDOW_TICKS  (1000)   /* 1s 内未汇报视为任务卡死 */

void Health_Init(void);
void Health_Register(uint8_t id);
void Health_Report(uint8_t id);          /* 任务周期性调用，标记存活 */
void Health_Check(void);                  /* 由监控任务调用，检测卡死并记错 */
uint8_t Health_IsAlive(uint8_t id);
uint32_t Health_UptimeSec(void);

#endif /* HEALTH_H */
