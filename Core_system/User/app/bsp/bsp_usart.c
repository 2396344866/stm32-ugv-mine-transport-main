/**
  * @file    bsp_usart.c
  * @brief   通用 UART 驱动：USART1/2/3 环形缓冲接收 + 发送，
  *          支持 USART3 部分重映射（PC10/PC11，用于 UWB BU03 模块），
  *          并通过 _write 将 printf 重定向到调试串口(USART2)。
  *
  *  设计要点：中断仅将字节压入环形缓冲并释放信号量（底半部），
  *          解析在任务上下文进行，避免中断中做 AT 指令解析。
  */
#include "bsp_usart.h"
#include "misc.h"
#include "task.h"
#include "board_config.h"
#include <stdio.h>

#define USART_NUM  3
#define RX_BUF_LEN 256

static uint8_t   s_rxbuf[USART_NUM][RX_BUF_LEN];
static volatile uint16_t s_rxhead[USART_NUM];   /* 写指针 (ISR) */
static volatile uint16_t s_rxtail[USART_NUM];   /* 读指针 (任务) */
static SemaphoreHandle_t s_xRxSem[USART_NUM];

static int usart_index(USART_TypeDef* u)
{
    if (u == USART1) return 0;
    if (u == USART2) return 1;
    if (u == USART3) return 2;
    return 0;
}

void BSP_USART_Init(USART_TypeDef* usart, uint32_t baud, uint8_t remap)
{
    int i = usart_index(usart);
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    /* 时钟 */
    if (usart == USART1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    }
    else if (usart == USART2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    }
    else /* USART3 */
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
        if (remap) GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
    }

    /* 引脚 */
    if (usart == USART1)
    {
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    else if (usart == USART2)
    {
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    else
    {
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
    }

    USART_InitStructure.USART_BaudRate            = baud;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(usart, &USART_InitStructure);

    USART_ITConfig(usart, USART_IT_RXNE, ENABLE);
    USART_Cmd(usart, ENABLE);

    uint8_t irq = (usart == USART1) ? USART1_IRQn : (usart == USART2) ? USART2_IRQn : USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannel                   = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;  /* >=11, 可调用 FreeRTOS API */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    s_rxhead[i] = s_rxtail[i] = 0;
    if (s_xRxSem[i] == NULL) s_xRxSem[i] = xSemaphoreCreateBinary();
}

void BSP_USART_SendString(USART_TypeDef* usart, const char* str)
{
    while (*str) BSP_USART_Send(usart, (const uint8_t*)str, 1), str++;
}

void BSP_USART_Send(USART_TypeDef* usart, const uint8_t* buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        USART_SendData(usart, buf[i]);
        while (USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET);
    }
}

uint16_t BSP_USART_ReadAvail(USART_TypeDef* usart, uint8_t* buf, uint16_t maxlen)
{
    int i = usart_index(usart);
    uint16_t got = 0;
    taskENTER_CRITICAL();
    while ((s_rxtail[i] != s_rxhead[i]) && (got < maxlen))
    {
        buf[got++] = s_rxbuf[i][s_rxtail[i]];
        s_rxtail[i] = (s_rxtail[i] + 1) & (RX_BUF_LEN - 1);
    }
    taskEXIT_CRITICAL();
    return got;
}

int BSP_USART_Read(USART_TypeDef* usart, uint8_t* buf, uint16_t maxlen, uint32_t timeout_ms)
{
    int i = usart_index(usart);
    uint32_t t0 = xTaskGetTickCount();
    uint16_t got = 0;

    while (got < maxlen)
    {
        if (s_rxtail[i] != s_rxhead[i])
        {
            buf[got++] = s_rxbuf[i][s_rxtail[i]];
            s_rxtail[i] = (s_rxtail[i] + 1) & (RX_BUF_LEN - 1);
            continue;
        }
        /* 暂无数据：等待信号量（新数据到达）或超时 */
        if (timeout_ms == 0) break;
        uint32_t elapsed = (xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
        if (elapsed >= timeout_ms) break;
        if (s_xRxSem[i])
        {
            uint32_t remain = timeout_ms - elapsed;
            xSemaphoreTake(s_xRxSem[i], pdMS_TO_TICKS(remain));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    return (int)got;
}

void BSP_USART_IRQHandler(USART_TypeDef* usart)
{
    int i = usart_index(usart);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (USART_GetITStatus(usart, USART_IT_RXNE) != RESET)
    {
        uint8_t d = (uint8_t)USART_ReceiveData(usart);
        uint16_t next = (s_rxhead[i] + 1) & (RX_BUF_LEN - 1);
        if (next != s_rxtail[i])
        {
            s_rxbuf[i][s_rxhead[i]] = d;
            s_rxhead[i] = next;
        }
        USART_ClearITPendingBit(usart, USART_IT_RXNE);
        if (s_xRxSem[i]) xSemaphoreGiveFromISR(s_xRxSem[i], &xHigherPriorityTaskWoken);
    }

    /* 清除过载/帧错误等，避免中断死锁 */
    if (USART_GetITStatus(usart, USART_IT_ORE) != RESET)
    {
        USART_ReceiveData(usart);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ---------- printf 重定向（newlib-nano _write -> 调试串口 USART2） ---------- */
int _write(int fd, char* ptr, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++)
    {
        USART_SendData(DBG_USART, (uint8_t)ptr[i]);
        while (USART_GetFlagStatus(DBG_USART, USART_FLAG_TXE) == RESET);
    }
    return len;
}
