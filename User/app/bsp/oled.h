#ifndef __OLED_H
#define __OLED_H
#include "stm32f10x.h"
void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowFloat(uint8_t Line, uint8_t Column, float Number, uint8_t Length, uint8_t Decimals);
void OLED_ShowSignedFloat(uint8_t Line, uint8_t Column, float Number, uint8_t Length, uint8_t Decimals);
void OLED_ShowNum_pointer(uint8_t Line, uint8_t Column, uint8_t *Number);
void OLED_ShowChar_Chinese_16x16(uint8_t Line, uint8_t Column, uint8_t Char);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void OLED_ShowImageBMG(void);
#endif
