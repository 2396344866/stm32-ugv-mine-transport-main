#include "LED.h"

/**
  * @brief  LED 初始化 (蓝=PC13, 绿=PC14, 推挽输出, 默认灭)
  */
void LED_GPIO_Config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_13 | GPIO_Pin_14);   /* 默认高电平 = 灭 */
}

void LED_Blue(uint8_t on)
{
    if (on) GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    else    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void LED_Green(uint8_t on)
{
    if (on) GPIO_ResetBits(GPIOC, GPIO_Pin_14);
    else    GPIO_SetBits(GPIOC, GPIO_Pin_14);
}
