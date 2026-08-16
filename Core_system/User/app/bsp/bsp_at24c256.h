#ifndef BSP_AT24C256_H
#define BSP_AT24C256_H

#include "stm32f10x.h"

/* AT24C256 (32KB) EEPROM，挂载于 I2C2（与 MPU6050 同总线，地址 0xAE=0x57<<1）
 * 用于监测数据本地断点续传存储（高稳定性设计）。 */
void BSP_AT24C256_Init(void);
int  BSP_AT24C256_Write(uint16_t addr, const uint8_t* data, uint16_t len);
int  BSP_AT24C256_Read(uint16_t addr, uint8_t* data, uint16_t len);

#endif /* BSP_AT24C256_H */
