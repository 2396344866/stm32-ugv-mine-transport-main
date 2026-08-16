#ifndef BSP_RTC_H
#define BSP_RTC_H

#include "stm32f10x.h"

/* RTC（LSI）+ 低功耗调度（高稳定性设计）
 * 用于监测子系统非阻塞低功耗唤醒，替代忙等轮询。 */
int  BSP_RTC_Init(void);
void BSP_RTC_GetTime(uint8_t* hour, uint8_t* min, uint8_t* sec);
int  BSP_RTC_SetWakeupSeconds(uint32_t sec);  /* 配置 RTC 闹钟唤醒 */
void BSP_EnterStopUntilWakeup(void);           /* 进入 STOP，等待 RTC 唤醒后恢复时钟 */

#endif /* BSP_RTC_H */
