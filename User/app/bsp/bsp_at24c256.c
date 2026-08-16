/**
  * @file    bsp_at24c256.c
  * @brief   AT24C256 EEPROM 驱动（I2C2, PB10/PB11, 16 位地址 + 写后 ACK 轮询）
  */
#include "bsp_at24c256.h"
#include "board_config.h"
#include <string.h>

#define AT24_PAGE_SIZE  64
#define I2C_TMO         100000

static void I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t ev)
{
    uint32_t t = I2C_TMO;
    while (I2C_CheckEvent(I2Cx, ev) != SUCCESS && t--) ;
}

static void I2C_AckPoll(I2C_TypeDef* I2Cx, uint8_t addr7)
{
    /* 写完成后轮询设备是否就绪（ACK） */
    uint32_t t = I2C_TMO;
    do {
        I2C_GenerateSTART(I2Cx, ENABLE);
        while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_SB) == RESET && t--) ;
        I2C_Send7bitAddress(I2Cx, addr7 << 1, I2C_Direction_Transmitter);
    } while ((I2C_GetFlagStatus(I2Cx, I2C_FLAG_ADDR) == RESET) && t--);
    I2C_GenerateSTOP(I2Cx, ENABLE);
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
    while (written < len)
    {
        uint16_t page_off = addr & (AT24_PAGE_SIZE - 1);
        uint16_t chunk = AT24_PAGE_SIZE - page_off;
        if (chunk > (len - written)) chunk = len - written;

        I2C_GenerateSTART(I2C2, ENABLE);
        I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
        I2C_Send7bitAddress(I2C2, AT24C256_ADDR, I2C_Direction_Transmitter);
        I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
        I2C_SendData(I2C2, (uint8_t)(addr >> 8));
        I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        I2C_SendData(I2C2, (uint8_t)(addr & 0xFF));
        I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

        for (uint16_t i = 0; i < chunk; i++)
        {
            I2C_SendData(I2C2, data[written + i]);
            I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
        }
        I2C_GenerateSTOP(I2C2, ENABLE);
        I2C_AckPoll(I2C2, AT24C256_ADDR);

        addr += chunk;
        written += chunk;
    }
    return 0;
}

int BSP_AT24C256_Read(uint16_t addr, uint8_t* data, uint16_t len)
{
    if (len == 0) return 0;

    /* 写地址相位 */
    I2C_GenerateSTART(I2C2, ENABLE);
    I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C2, AT24C256_ADDR, I2C_Direction_Transmitter);
    I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    I2C_SendData(I2C2, (uint8_t)(addr >> 8));
    I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    I2C_SendData(I2C2, (uint8_t)(addr & 0xFF));
    I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);

    /* 重复起始 + 读相位 */
    I2C_GenerateSTART(I2C2, ENABLE);
    I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
    I2C_Send7bitAddress(I2C2, AT24C256_ADDR, I2C_Direction_Receiver);
    I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);

    for (uint16_t i = 0; i < len; i++)
    {
        if (i == (len - 1))
        {
            I2C_AcknowledgeConfig(I2C2, DISABLE);
        }
        I2C_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
        data[i] = I2C_ReceiveData(I2C2);
    }
    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return 0;
}
