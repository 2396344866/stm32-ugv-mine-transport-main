#ifndef __MPU6050_REG_H
#define __MPU6050_REG_H

/* MPU6050 寄存器与板级 I2C 定义（与 STM32F103RCT6 工程统一）
 * 供 MPU6050.c 与 inv_mpu Motion Driver 使用。
 */
#include "stm32f10x.h"

/* ---- 硬件接口（I2C2, PB10/PB11，与 AT24C256 共享总线） ---- */
#define MPU6050_I2Cx            I2C2
#define MPU6050_GPIO_PORT       GPIOB
#define MPU6050_SCL_PIN         GPIO_Pin_10
#define MPU6050_SDA_PIN         GPIO_Pin_11
#define MPU6050_SLAVE_ADDRESS   (0x68)        /* AD0=0 */
#define MPU6050_I2C_SPEED       (100000)      /* 标准模式 100kHz */
#define MPU6050_I2C_DUTY_CYCLE  I2C_DutyCycle_2

/* ---- I2C 等待超时（MPU6050_WaitEvent 使用） ---- */
#define I2C_TIMEOUT             (0x5000)

/* ---- 寄存器地址 ---- */
#define WHO_AM_I_ADDRESS        (0x75)
#define PWR_MGMT_1_ADDRESS      (0x6B)
#define PWR_MGMT_2_ADDRESS      (0x6C)
#define SMPLRT_DIV_ADDRESS      (0x19)
#define CONFIG_DLPF_ADDRESS     (0x1A)
#define GYRO_FS_SEL_ADDRESS     (0x1B)
#define ACCEL_FS_SEL_ADDRESS    (0x1C)
#define FIFO_EN_ADDRESS         (0x23)
#define INT_ENABLE_ADDRESS      (0x38)
#define INT_STATUS_ADDRESS      (0x3A)
#define ACCEL_XOUT_H_ADDRESS    (0x3B)
#define TEMP_OUT_H_ADDRESS      (0x41)
#define GYRO_XOUT_H_ADDRESS     (0x43)
#define USER_CTRL_ADDRESS       (0x6A)
#define FIFO_COUNTH_ADDRESS     (0x72)
#define FIFO_R_W_ADDRESS        (0x74)

/* ---- 配置值 ---- */
#define PWR_MGMT_1_CONFIG       (0x00)   /* 退出睡眠，内部时钟 */
#define PWR_MGMT_2_CONFIG       (0x00)   /* 全部轴使能 */
#define SMPLRT_DIV_CONFIG       (0x04)   /* 1kHz/(1+4)=200Hz */
#define CONFIG_DLPF_CONFIG      (0x03)   /* DLPF 44Hz */
#define GYRO_FS_SEL_CONFIG      (0x18)   /* ±2000°/s */
#define ACCEL_FS_SEL_CONFIG     (0x00)   /* ±2g */

#endif /* __MPU6050_REG_H */
