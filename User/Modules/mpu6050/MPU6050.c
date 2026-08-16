/**
  ******************************************************************************
  * @file    MPU6050.c
  * @author  YourName
  * @version V1.78
  * @brief   MPU6050六轴传感器驱动库
  ******************************************************************************
  * @attention
  * - 硬件平台: STM32F1
  * - 开发环境: Keil MDK-ARM V5
  * - 传感器量程: ±2g加速度 /±2000°/s陀螺仪（可修改）
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"  
#include <math.h>
#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif
#include <stdio.h>
#include "MPU6050.h"
#include "MPU6050_Reg.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "Delay.h"
/**
  * @brief  等待I2C事件完成
  * @param  I2Cx: I2C控制器(如I2C1/I2C2)
  * @param  I2C_EVENT: 等待的I2C事件
  * @retval 0: 事件正常
  *         1: 等待超时
  */

static int MPU6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
    uint32_t Timeout = I2C_TIMEOUT;                                 // 设定超时计数初始值
    // 循环等待指定的 I2C 事件发生
    while (I2C_CheckEvent(I2Cx, I2C_EVENT)!= SUCCESS)  
    {
        Timeout --;                                 // 每次循环计数值自减
        // 若计数值减到 0，表明等待超时
        if (Timeout == 0)                           
        {
            I2C_GenerateSTOP(MPU6050_I2Cx, ENABLE);
            return 1;                             // 跳出等待循环
        }
    }
    return 0;
}

/**
  * @brief  I2C连续写寄存器
  * @param  slave_addr: 从机地址(7位格式)
  * @param  reg_addr: 目标寄存器地址
  * @param  length: 数据长度
  * @param  data: 待发送数据缓冲区指针
  * @retval 0: 写入成功
  */
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    // 生成 I2C 起始信号
    I2C_GenerateSTART(MPU6050_I2Cx, ENABLE);
    // 等待主模式选择事件
    MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_MODE_SELECT);

    // 发送 7 位从设备地址，并设置为发送模式
    I2C_Send7bitAddress(MPU6050_I2Cx, slave_addr <<1, I2C_Direction_Transmitter);
    // 等待主发送器模式选择事件
    MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    // 发送寄存器地址
    I2C_SendData(MPU6050_I2Cx, reg_addr);
    // 等待字节发送事件
    MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    // 循环发送数据
    for(uint8_t i=0; i<length; i++){
        I2C_SendData(MPU6050_I2Cx, data[i]);
        // 等待字节发送事件
        MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    }
    // 生成 I2C 停止信号
    I2C_GenerateSTOP(MPU6050_I2Cx, ENABLE);
    return 0;
}
/**
  * @brief  I2C连续读寄存器
  * @param  slave_addr: 从机地址(7位格式)
  * @param  reg_addr: 目标寄存器地址
  * @param  length: 数据长度
  * @param  data: 接收数据缓冲区指针
  * @retval 0: 读取成功
  */
int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    // 生成 I2C 起始信号
    I2C_GenerateSTART(MPU6050_I2Cx, ENABLE);
    // 等待主模式选择事件
    MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
    // 发送 7 位从设备地址，并设置为发送模式
    I2C_Send7bitAddress(MPU6050_I2Cx, slave_addr<<1, I2C_Direction_Transmitter);
    // 等待主发送器模式选择事件
    MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
    // 发送寄存器地址
    I2C_SendData(MPU6050_I2Cx, reg_addr);
    // 等待字节发送完成事件
    MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
    // 再次生成 I2C 起始信号
    I2C_GenerateSTART(MPU6050_I2Cx, ENABLE);
    // 等待主模式选择事件
    MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
    // 发送 7 位从设备地址，并设置为接收模式
    I2C_Send7bitAddress(MPU6050_I2Cx, slave_addr<<1, I2C_Direction_Receiver);
    // 等待主接收器模式选择事件
    MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
    // 循环接收数据
    for(uint8_t i = 0; i < length; i++){
        if(i == length - 1){
            // 在接收最后一个字节时，禁用应答并生成停止信号
            I2C_AcknowledgeConfig(MPU6050_I2Cx, DISABLE);
            I2C_GenerateSTOP(MPU6050_I2Cx, ENABLE);
        }
        // 等待字节接收事件
        MPU6050_WaitEvent(MPU6050_I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);
        data[i] = I2C_ReceiveData(MPU6050_I2Cx);
    }
    // 恢复应答配置为使能
    I2C_AcknowledgeConfig(MPU6050_I2Cx, ENABLE);
    return 0;
}
/**
 * @brief 定义一个表示 I2C 总线锁的变量
 * 用于控制对 I2C 总线的访问
 */
volatile uint8_t i2c_bus_lock = 0;
/**
  * @brief  写MPU6050寄存器
  * @param  RegAddress: 寄存器地址
  * @param  Data: 待写入数据
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    // 等待 I2C 总线空闲
    while(i2c_bus_lock);  
    i2c_bus_lock = 1;	
    // 通过 Sensors_I2C_WriteRegister 函数向指定寄存器写入单字节数据
    Sensors_I2C_WriteRegister(MPU6050_SLAVE_ADDRESS, RegAddress, 1, &Data);  
    i2c_bus_lock = 0;
}
/**
  * @brief  写MPU6050寄存器
  * @param  RegAddress: 寄存器地址
  * @param  Data: 待写入数据
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    // 等待 I2C 总线空闲
    while(i2c_bus_lock);
    i2c_bus_lock = 1;
    uint8_t data;
    // 通过 Sensors_I2C_ReadRegister 函数从指定寄存器读取单字节数据
    Sensors_I2C_ReadRegister(MPU6050_SLAVE_ADDRESS, RegAddress, 1, &data);  
    i2c_bus_lock = 0;  
    return data;
}


/**
 * @brief 获取 MPU6050 的设备 ID 号
 * 
 * @param 无
 * @return MPU6050 的 ID 号
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(WHO_AM_I_ADDRESS);  // 返回 WHO_AM_I 寄存器的值
}
/*
当前代码未涉及温度传感器数据的读取。
如果需要，可以通过以下寄存器实现：
uint16_t temp = (MPU6050_ReadReg(MPU6050_TEMP_OUT_H) << 8) |MPU6050_ReadReg(MPU6050_TEMP_OUT_L);
*/

/**
 * @brief 初始化 MPU6050 设备
 * 
 * @param 无
 * @return 无
 */
 void I2C_GPIO_Config(void) 
 {
    /* 开启相关时钟 */
    // 开启 I2C2 的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);  
    // 开启 MPU6050_GPIO_PORT  的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
    /* GPIO 初始化 */
    GPIO_InitTypeDef GPIO_InitStructure;
    // 设置 GPIO 模式为复用开漏输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;  
    // 选择 PB10 和 PB11 引脚
    GPIO_InitStructure.GPIO_Pin = MPU6050_SDA_PIN | MPU6050_SCL_PIN;  
    // 设置 GPIO 速度为 50MHz
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    // 初始化 MPU6050_GPIO_PORT 
    GPIO_Init(MPU6050_GPIO_PORT , &GPIO_InitStructure);  
}

void I2C_Config(void) {
    /* I2C 初始化 */
    I2C_InitTypeDef I2C_InitStructure;  // 定义结构体变量
    // 选择 I2C 模式
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;  
    // 设置时钟速度为 50KHz
    I2C_InitStructure.I2C_ClockSpeed = MPU6050_I2C_SPEED;  
    // 选择时钟占空比为 Tlow/Thigh = 2
    I2C_InitStructure.I2C_DutyCycle = MPU6050_I2C_DUTY_CYCLE;  
    // 使能应答
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;  
    // 选择 7 位应答地址
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;  
    // 设置自身地址（从机模式下有效）
    I2C_InitStructure.I2C_OwnAddress1 = PWR_MGMT_2_CONFIG;  
    // 配置 I2C2
    I2C_Init(MPU6050_I2Cx, &I2C_InitStructure);  
    /* 使能 I2C2 */
    I2C_Cmd(MPU6050_I2Cx, ENABLE);  // 使能 I2C2 开始运行
}


void MPU6050_device_config(void)
{   
    /* MPU6050 寄存器初始化，根据 MPU6050 手册配置重要寄存器 */
    // 电源管理寄存器 1，取消睡眠模式，选择时钟源为 X 轴陀螺仪
    MPU6050_WriteReg(PWR_MGMT_1_ADDRESS, PWR_MGMT_1_CONFIG);		
    // 电源管理寄存器 2，保持默认值 0，所有轴均不待机
    MPU6050_WriteReg(PWR_MGMT_2_ADDRESS, PWR_MGMT_2_CONFIG);  
    // 采样率分频寄存器，配置采样率
    MPU6050_WriteReg(SMPLRT_DIV_ADDRESS, SMPLRT_DIV_CONFIG); 
    // 配置寄存器，配置 DLPF
    MPU6050_WriteReg(CONFIG_DLPF_ADDRESS, CONFIG_DLPF_CONFIG);  
    // 陀螺仪配置寄存器，选择满量程为 ±2000°/s
    MPU6050_WriteReg(GYRO_FS_SEL_ADDRESS, GYRO_FS_SEL_CONFIG);    
    // 加速度计配置寄存器，选择满量程为 ±2g
    MPU6050_WriteReg(ACCEL_FS_SEL_ADDRESS, ACCEL_FS_SEL_CONFIG);  
}
void MPU6050_Init(void)
{
    // GPIO 配置
    I2C_GPIO_Config();
    // IIC控制
    I2C_Config();

}


/**
 * @brief 初始化MPU6050的数字运动处理器（DMP）
 * 
 * 该函数用于对MPU6050传感器的DMP进行初始化配置，包括MPU本身的初始化、传感器设置、
 * FIFO配置、DMP固件加载、方向设置、功能启用、采样率设置以及最终启用DMP。
 * 
 * @return 无
 */
void MPU6050_DMP_Init(void) {
    MPU6050_device_config(); // 确保基础配置
    // 初始化MPU6050传感器
    // 调用mpu_init函数进行MPU6050的基本初始化，传入NULL表示使用默认配置
    mpu_init(NULL);
    // 禁用旁路模式
    // 如果旁路模式设置为0，则不链接外部磁力计
    // 这里注释掉了mpu_set_bypass(0)，若需要禁用旁路模式可取消注释
    // mpu_set_bypass(0);
    // 设置MPU6050要使用的传感器
    // INV_XYZ_GYRO表示启用三轴陀螺仪，INV_XYZ_ACCEL表示启用三轴加速度计
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    // 配置FIFO（First In First Out）缓冲区
    // 将陀螺仪和加速度计的数据存储到FIFO缓冲区中
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    // 设置低通滤波器为42Hz（与DMP配合）
    mpu_set_lpf(42);
    // 设置陀螺仪的满量程范围
    // 将陀螺仪的满量程范围设置为2000°/s
    mpu_set_gyro_fsr(500); 
    // 设置加速度计的满量程范围
    // 将加速度计的满量程范围设置为2g
    mpu_set_accel_fsr(4);
    // 加载DMP的运动驱动固件
    // 使MPU6050能够使用DMP的功能
    if (dmp_load_motion_driver_firmware() != 0) {
        printf("DMP Firmware Load Failed!\n");
        while(1);
    }
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    // 启用DMP的特定功能
    // DMP_FEATURE_6X_LP_QUAT表示启用6轴低功耗四元数计算
    // DMP_FEATURE_SEND_RAW_GYRO 表示添加陀螺仪原始数据输出
    // DMP_FEATURE_SEND_RAW_ACCEL表示发送原始加速度计数据
    // DMP_FEATURE_GYRO_CAL表示启用陀螺仪校准功能
		dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | 
                  DMP_FEATURE_SEND_RAW_ACCEL |
                  DMP_FEATURE_SEND_RAW_GYRO |
                  DMP_FEATURE_TAP |          // 添加轻敲检测
                  DMP_FEATURE_GYRO_CAL);
    /*参数：rate_div
        设置DMP的FIFO采样率
        FIFO 输出速率 = DMP_SAMPLE_RATE / (rate_div + 1)
    */
    mpu_set_sample_rate(200); // 设置采样率
    dmp_set_fifo_rate(200);  // 与DMP采样率匹配
    // 启用DMP
    // 将DMP状态设置为启用，使DMP开始工作
    mpu_set_dmp_state(1);   
    // 等待DMP校准完成
    Delay_ms(500);
    // 清空FIFO
    mpu_reset_fifo();		
}

/**
 * @brief 从 DMP FIFO 读取四元数并转换为欧拉角(度)
 * @param pitch 俯仰角(度)
 * @param roll  滚转角(度)
 * @param yaw   偏航角(度)
 * @return 0 成功, 非0 失败(无四元数数据/读取错误)
 */
int MPU6050_DMP_GetData(float *pitch, float *roll, float *yaw)
{
    short gyro[3], accel[3];
    long  quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    int  ret = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
    if (ret != 0) return ret;
    if (!(sensors & INV_WXYZ_QUAT)) return -1;

    float ypr[3];   /* ypr[0]=yaw, [1]=pitch, [2]=roll */
    quat_to_euler(quat, ypr);
    if (yaw)   *yaw   = ypr[0];
    if (pitch) *pitch = ypr[1];
    if (roll)  *roll  = ypr[2];
    return 0;
}



/*下面同样为DMP初始化 用于出现问题，调试*/
//void MPU6050_DMP_Init(void) {
//    int result;

//    // MPU初始化
//    result = mpu_init(NULL);
//    if (result == 0) {
//        printf("mpu_init OK\r\n");

//        // 开启传感器
//        result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//        if (result == 0) {
//            printf("Sensors Set Success\r\n");

//            // 配置FIFO
//            result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
//            if (result == 0) {
//                printf("FIFO Config Success\r\n");

//                // 设置陀螺仪满量程范围
//                result = mpu_set_gyro_fsr((unsigned short)2000);
//                if (result == 0) {
//                    printf("Gyro FSR Set Success\r\n");

//                    // 设置加速度计满量程范围
//                    result = mpu_set_accel_fsr((unsigned char)2);
//                    if (result == 0) {
//                        printf("Accel FSR Set Success\r\n");

//                        // 加载DMP固件
//                        result = dmp_load_motion_driver_firmware();
//                        if (result == 0) {
//                            printf("DMP Firmware Load Success\r\n");

//                            // 设置方向
//                            result = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
//                            if (result == 0) {
//                                printf("DMP Orientation Set Success\r\n");

//                                // 启用DMP功能
//                                result = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_GYRO_CAL);
//                                if (result == 0) {
//                                    printf("DMP Feature Enable Success\r\n");

//                                    // 设置FIFO采样率
//                                    dmp_set_fifo_rate(100);  // 100Hz采样率
//                                    // 启用DMP
//                                    mpu_set_dmp_state(1);
//                                } else {
//                                    printf("dmp_enable_feature failed: %d\r\n", result);
//                                }
//                            } else {
//                                printf("dmp_set_orientation failed: %d\r\n", result);
//                            }
//                        } else {
//                            printf("dmp_load_motion_driver_firmware failed: %d\r\n", result);
//                        }
//                    } else {
//                        printf("mpu_set_accel_fsr error: %d\r\n", result);
//                    }
//                } else {
//                    printf("mpu_set_gyro_fsr error: %d\r\n", result);
//                }
//            } else {
//                printf("mpu_configure_fifo failed: %d\r\n", result);
//            }
//        } else {
//            printf("mpu_set_sensors failed: %d\r\n", result);
//        }
//    } else {
//        printf("MPU Init error\r\n");
//    }
//}

/**
 * @brief 将Q30格式的四元数转换为欧拉角（单位：度）
 * 
 * 该函数接收一个Q30格式的四元数数组，并将其转换为欧拉角（偏航角Yaw、俯仰角Pitch、滚转角Roll）。
 * 转换过程包括将Q30格式的四元数转换为浮点数、对四元数进行归一化处理、将归一化后的四元数转换为欧拉角，
 * 最后将欧拉角从弧度转换为角度。
 * 
 * @param quat 指向存储Q30格式四元数的长整型数组的指针，数组长度应为4，分别存储q0, q1, q2, q3。
 * @param ypr  指向存储欧拉角的浮点型数组的指针，数组长度应为3，分别存储偏航角（Yaw）、俯仰角（Pitch）、滚转角（Roll）。
 */
void quat_to_euler(long *quat, float *ypr) {
    // 1. 将Q30格式四元数转换为浮点（范围：-2.0 ~ +2.0）
    // Q30格式是一种定点数表示法，这里将其转换为浮点数，通过除以2的30次方（1073741824）来实现
    float q0 = quat[0] / 1073741824.0f;  
    float q1 = quat[1] / 1073741824.0f;
    float q2 = quat[2] / 1073741824.0f;
    float q3 = quat[3] / 1073741824.0f;
    // 2. 四元数归一化（DMP输出的四元数通常已归一化，但建议添加）
    // 计算四元数的模长，确保四元数是单位四元数，以保证后续转换的准确性
    float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
    // 3. 四元数转欧拉角（顺序：Yaw(Z), Pitch(Y), Roll(X)）
    // 根据四元数到欧拉角的转换公式，计算偏航角（Yaw）
    ypr[0] = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3);  
    // 根据四元数到欧拉角的转换公式，计算俯仰角（Pitch）
    ypr[1] = -asinf(2.0f * (q1 * q3 - q0 * q2));                               
    // 根据四元数到欧拉角的转换公式，计算滚转角（Roll）
    ypr[2] = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3); 
    // 4. 弧度转角度（M_PI 由 <math.h> 提供）
    // 将计算得到的欧拉角从弧度转换为角度，方便实际使用
    ypr[0] *= (180.0f / M_PI);
    ypr[1] *= (180.0f / M_PI);
    ypr[2] *= (180.0f / M_PI);
}
/*
//如果需要添加中断下面中断是GPIOB12
void EXTI_Config(void) {
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 配置PB12为中断输入
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;  // 上拉输入
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 映射PB12到EXTI12
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);

    // 配置EXTI为上升沿触发
    EXTI_InitStruct.EXTI_Line = EXTI_Line12;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);

    // 配置NVIC
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

// 中断服务函数
void EXTI15_10_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        dmp_data_ready = 1;  // 设置标志位
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
*/
unsigned char dmp_data_ready=1;
short gyro[3];          // 存储陀螺仪数据
short accel[3];         // 存储加速度计数据
long quat[4];           // 存储四元数数据
unsigned long timestamp; // 存储时间戳
short sensors;          // 存储传感器标志
unsigned char more;     // 存储剩余数据包数量
float ypr[3];
