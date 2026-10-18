/**
  * @file    bsp_at24c256.c
  * @brief   AT24C256 EEPROM 驱动（I2C2, PB10/PB11, 16 位地址 + 写后 ACK 轮询）
  */
#include "bsp_at24c256.h"
#include "board_config.h"
#include <string.h>
#include <stddef.h>

#define AT24_PAGE_SIZE  64
#define I2C_TMO         100000

/* 返回 SUCCESS/ERROR，取代原来的 void，使上层能真正感知总线超时 */
static int I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t ev)
{
    uint32_t t = I2C_TMO;
    while (I2C_CheckEvent(I2Cx, ev) != SUCCESS && t--) ;
    return (t == 0) ? ERROR : SUCCESS;
}

/* 写周期轮询：反复发 START + 地址，直到器件回 ACK（内部写周期 tWR 结束）为止。
 * addr7w = 已含 R/W 位的 8 位地址（写=0xAE，读=0xAF），与 I2C_Send7bitAddress
 * 的入参约定一致；本函数不再左移，否则会二次移位打到错误从机地址。
 * 返回 1 = 本次已收到 ACK，0 = 轮询超时（器件无响应）。 */
static uint8_t I2C_AckPoll(I2C_TypeDef* I2Cx, uint8_t addr7w)
{
    uint32_t t = I2C_TMO;
    while (t--)
    {
        I2C_GenerateSTART(I2Cx, ENABLE);
        if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
        {
            I2C_GenerateSTOP(I2Cx, ENABLE);
            continue;
        }
        I2C_Send7bitAddress(I2Cx, addr7w, I2C_Direction_Transmitter);
        if (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == SUCCESS)
        {
            I2C_GenerateSTOP(I2Cx, ENABLE);
            return 1;      /* 器件已就绪 */
        }
        I2C_GenerateSTOP(I2Cx, ENABLE);
    }
    return 0;
}

void BSP_AT24C256_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;  /* PB10 SCL, PB11 SDA */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitTypeDef I2C_InitStructure;
    I2C_InitStructure.I2C_Mode               = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle          = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1        = 0x00;
    I2C_InitStructure.I2C_Ack                = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed         = 100000;
    I2C_Init(I2C2, &I2C_InitStructure);
    I2C_Cmd(I2C2, ENABLE);
}

int BSP_AT24C256_Write(uint16_t addr, const uint8_t* data, uint16_t len)
{
    uint16_t written = 0;
    if (data == NULL) return -1;

    while (written < len)
    {
        /* AT24C256 每 64 字节一页，页内地址自动回卷，故必须按页边界切块，
         * 否则跨页的长写会在页末回卷覆写同页起始地址。 */
        uint16_t page_off = addr & (AT24_PAGE_SIZE - 1);
        uint16_t chunk = AT24_PAGE_SIZE - page_off;
        if (chunk > (len - written)) chunk = len - written;

        I2C_GenerateSTART(I2C2, ENABLE);
        if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
        {
            I2C_GenerateSTOP(I2C2, ENABLE);
            return -1;
        }
        I2C_Send7bitAddress(I2C2, AT24C256_ADDR, I2C_Direction_Transmitter);
        if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
        {
            I2C_GenerateSTOP(I2C2, ENABLE);
            return -1;
        }
        /* 16 位字地址：高字节先发 */
        I2C_SendData(I2C2, (uint8_t)(addr >> 8));
        if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        {
            I2C_GenerateSTOP(I2C2, ENABLE);
            return -1;
        }
        I2C_SendData(I2C2, (uint8_t)(addr & 0xFF));
        if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
        {
            I2C_GenerateSTOP(I2C2, ENABLE);
            return -1;
        }

        for (uint16_t i = 0; i < chunk; i++)
        {
            I2C_SendData(I2C2, data[written + i]);
            if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
            {
                I2C_GenerateSTOP(I2C2, ENABLE);
                return -1;
            }
        }
        I2C_GenerateSTOP(I2C2, ENABLE);

        /* 必须等器件内部写周期（tWR ≈ 5ms）结束，否则下一页写入会被 NACK */
        if (I2C_AckPoll(I2C2, AT24C256_ADDR) == 0)
        {
            return -1;
        }

        addr += chunk;
        written += chunk;
    }
    return 0;
}

int BSP_AT24C256_Read(uint16_t addr, uint8_t* data, uint16_t len)
{
    if (len == 0) return 0;
    if (data == NULL) return -1;

    /* 写地址相位 */
    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        return -1;
    }
    I2C_Send7bitAddress(I2C2, AT24C256_ADDR, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        return -1;
    }
    I2C_SendData(I2C2, (uint8_t)(addr >> 8));
    if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        return -1;
    }
    I2C_SendData(I2C2, (uint8_t)(addr & 0xFF));
    if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        return -1;
    }

    /* 重复起始 + 读相位 */
    I2C_GenerateSTART(I2C2, ENABLE);
    if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        return -1;
    }
    I2C_Send7bitAddress(I2C2, AT24C256_ADDR, I2C_Direction_Receiver);
    if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
    {
        I2C_GenerateSTOP(I2C2, ENABLE);
        return -1;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        if (i == (len - 1))
        {
            I2C_AcknowledgeConfig(I2C2, DISABLE);
        }
        if (I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
        {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2C_AcknowledgeConfig(I2C2, ENABLE);
            return -1;
        }
        data[i] = I2C_ReceiveData(I2C2);
    }
    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return 0;
}
