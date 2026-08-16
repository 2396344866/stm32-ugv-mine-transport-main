#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

void LED_GPIO_Config(void);
void LED_Blue(uint8_t on);
void LED_Green(uint8_t on);

#endif
