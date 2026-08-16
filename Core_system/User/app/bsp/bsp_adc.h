#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "stm32f10x.h"

/* 工业管道气压：扩散硅压力变送器 -> ADC1_CH4(PA4)，硬件 RC 低通 + TVS */
void BSP_ADC_Init(void);
uint16_t BSP_ADC_ReadRaw(void);   /* 单次软件触发转换，12 位原始值 0..4095 */

#endif /* BSP_ADC_H */
