#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H

#include "stm32f10x.h"

typedef enum
{
    MOTOR_FWD = 0,
    MOTOR_REV = 1,
    MOTOR_STOP = 2
} MotorDir;

/* 电机 PWM(TIM1_CH1/PA8) + 方向(PB12/PB13) + 编码器(TIM2/PA0/PA1) + 状态指示(PB0/1/2) */
void BSP_Motor_Init(void);
void BSP_Motor_SetDuty(uint16_t duty);     /* 0..999 */
void BSP_Motor_SetDir(MotorDir dir);
void BSP_Encoder_Reset(void);
int16_t BSP_Encoder_ReadDelta(void);        /* 自上次调用以来的计数增量 */
void BSP_Status_Set(uint8_t warn, uint8_t buz, uint8_t err);

#endif /* BSP_MOTOR_H */
