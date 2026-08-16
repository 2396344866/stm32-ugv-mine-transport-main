#ifndef BSP_USART_H
#define BSP_USART_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"

/**
  * @brief  初始化指定 USART（TX 复用推挽 / RX 浮空），开启 RXNE 中断。
  * @param  usart  USART1/USART2/USART3
  * @param  baud   波特率
  * @param  remap  仅 USART3 有效：1=部分重映射(PC10/PC11)
  */
void BSP_USART_Init(USART_TypeDef* usart, uint32_t baud, uint8_t remap);

/* 发送 */
void BSP_USART_SendString(USART_TypeDef* usart, const char* str);
void BSP_USART_Send(USART_TypeDef* usart, const uint8_t* buf, uint16_t len);

/* 非阻塞读取：拷贝当前环形缓冲区中可用字节，返回拷贝数量 */
uint16_t BSP_USART_ReadAvail(USART_TypeDef* usart, uint8_t* buf, uint16_t maxlen);

/* 带超时读取（等待信号量），返回实际读取字节数；timeout_ms=0 表示不等待 */
int BSP_USART_Read(USART_TypeDef* usart, uint8_t* buf, uint16_t maxlen, uint32_t timeout_ms);

/* 由 stm32f10x_it.c 在各 USART 中断中调用 */
void BSP_USART_IRQHandler(USART_TypeDef* usart);

#endif /* BSP_USART_H */
