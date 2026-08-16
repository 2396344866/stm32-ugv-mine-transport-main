#include "Key.h"
#include "Delay.h"

/**
  * @brief  按键 GPIO 初始化 (KEY1=PC0, KEY2=PC1, 上拉输入)
  */
void KEY_GPIO_Config(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief  阻塞式按键扫描，返回键码 0/1/2
  * @note   带入机消抖；多键同时按下时返回最后匹配
  */
uint8_t Key_GetNum(void)
{
    uint8_t KeyNum = 0;

    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0) == 0);
        Delay_ms(20);
        KeyNum = 1;
    }

    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 0)
    {
        Delay_ms(20);
        while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) == 0);
        Delay_ms(20);
        KeyNum = 2;
    }

    return KeyNum;
}
