/**
  * @file    bsp_motor.c
  * @brief   控制子系统底层：电机 PWM、方向、编码器、状态指示
  */
#include "bsp_motor.h"
#include "board_config.h"

void BSP_Motor_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* 方向 + 状态指示：推挽输出，默认低 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = MOTOR_DIR_A_PIN | MOTOR_DIR_B_PIN |
                                    STATUS_WARN_PIN | STATUS_BUZ_PIN | STATUS_ERR_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOB, MOTOR_DIR_A_PIN | MOTOR_DIR_B_PIN |
                          STATUS_WARN_PIN | STATUS_BUZ_PIN | STATUS_ERR_PIN);

    /* PWM 输出 PA8 (TIM1_CH1) */
    GPIO_InitStructure.GPIO_Pin   = MOTOR_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(MOTOR_PWM_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period        = 999;
    TIM_TimeBaseStructure.TIM_Prescaler     = 71;    /* 72MHz/72 = 1MHz */
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse        = 0;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* 编码器接口 (TIM2, PA0/PA1) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = ENCODER_A_PIN | ENCODER_B_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENCODER_PORT, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period        = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

void BSP_Motor_SetDuty(uint16_t duty)
{
    if (duty > 999) duty = 999;
    TIM_SetCompare1(TIM1, duty);
}

void BSP_Motor_SetDir(MotorDir dir)
{
    if (dir == MOTOR_FWD)
    {
        GPIO_SetBits(MOTOR_DIR_A_PORT, MOTOR_DIR_A_PIN);
        GPIO_ResetBits(MOTOR_DIR_B_PORT, MOTOR_DIR_B_PIN);
    }
    else if (dir == MOTOR_REV)
    {
        GPIO_ResetBits(MOTOR_DIR_A_PORT, MOTOR_DIR_A_PIN);
        GPIO_SetBits(MOTOR_DIR_B_PORT, MOTOR_DIR_B_PIN);
    }
    else
    {
        GPIO_ResetBits(MOTOR_DIR_A_PORT, MOTOR_DIR_A_PIN);
        GPIO_ResetBits(MOTOR_DIR_B_PORT, MOTOR_DIR_B_PIN);
    }
}

void BSP_Encoder_Reset(void)
{
    TIM_SetCounter(TIM2, 0);
}

int16_t BSP_Encoder_ReadDelta(void)
{
    int16_t cnt = (int16_t)TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2, 0);
    return cnt;
}

void BSP_Status_Set(uint8_t warn, uint8_t buz, uint8_t err)
{
    if (warn) GPIO_SetBits(STATUS_WARN_PORT, STATUS_WARN_PIN); else GPIO_ResetBits(STATUS_WARN_PORT, STATUS_WARN_PIN);
    if (buz)  GPIO_SetBits(STATUS_BUZ_PORT,  STATUS_BUZ_PIN);  else GPIO_ResetBits(STATUS_BUZ_PORT,  STATUS_BUZ_PIN);
    if (err)  GPIO_SetBits(STATUS_ERR_PORT,  STATUS_ERR_PIN);  else GPIO_ResetBits(STATUS_ERR_PORT,  STATUS_ERR_PIN);
}
