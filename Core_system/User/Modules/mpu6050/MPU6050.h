#ifndef __MPU6050_H
#define __MPU6050_H
#include "stm32f10x.h"  
/**
 * @brief 初始化MPU6050传感器
 */
void MPU6050_Init(void);
/**
 * @brief 获取MPU6050的设备ID
 *
 * 该函数用于从MPU6050传感器读取其设备ID，通常用于验证传感器的连接和识别。
 *
 * @return uint8_t 成功读取时返回MPU6050的设备ID；若读取失败，返回值的含义取决于具体实现。
 */
uint8_t MPU6050_GetID(void);
/**
 * @brief 通过I2C接口向指定从设备的寄存器写入数据
 *
 * 该函数使用I2C接口向指定从设备的寄存器写入一定长度的数据。
 *
 * @param slave_addr 从设备的I2C地址。
 * @param reg_addr 要写入数据的寄存器地址。
 * @param length 要写入的数据长度（字节数）。
 * @param data 指向要写入的数据缓冲区的指针。
 * @return int 成功时返回写入操作的状态码（通常为0表示成功）；失败时返回错误码，具体含义取决于实现。
 */
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
/**
 * @brief 通过I2C接口从指定从设备的寄存器读取数据
 *
 * 该函数使用I2C接口从指定从设备的寄存器读取一定长度的数据，并将其存储到指定的数据缓冲区中。
 *
 * @param slave_addr 从设备的I2C地址。
 * @param reg_addr 要读取数据的寄存器地址。
 * @param length 要读取的数据长度（字节数）。
 * @param data 指向存储读取数据的缓冲区的指针。
 * @return int 成功时返回读取操作的状态码（通常为0表示成功）；失败时返回错误码，具体含义取决于实现。
 */
int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

/**
 * @brief 将四元数转换为欧拉角（偏航角、俯仰角、滚转角）
 *
 * 该函数将输入的四元数转换为对应的欧拉角（偏航角、俯仰角、滚转角），并将结果存储到对应的指针变量中。
 *
 * @param quat 指向存储四元数的常量长整型数组的指针，数组长度应为4。
 * @param yaw 指向存储偏航角结果的浮点型变量的指针。
 * @param pitch 指向存储俯仰角结果的浮点型变量的指针。
 * @param roll 指向存储滚转角结果的浮点型变量的指针。
 */
void quaternion_to_euler(const long *quat, float *yaw, float *pitch, float *roll);

/**
 * @brief 初始化MPU6050的数字运动处理器（DMP）
 *
 * 该函数对MPU6050的DMP进行初始化配置，包括加载固件、设置参数等操作，使DMP能够正常工作。
 */
void MPU6050_DMP_Init(void);

/**
 * @brief 从MPU6050的DMP获取欧拉角数据
 *
 * 该函数从MPU6050的数字运动处理器（DMP）中获取经过处理后的欧拉角（俯仰角、滚转角、偏航角）数据。
 *
 * @param pitch 指向存储俯仰角数据的浮点型变量的指针。
 * @param roll 指向存储滚转角数据的浮点型变量的指针。
 * @param yaw 指向存储偏航角数据的浮点型变量的指针。
 * @return int 成功时返回获取数据的状态码（通常为0表示成功）；失败时返回错误码，具体含义取决于实现。
 */
int MPU6050_DMP_GetData(float *pitch, float *roll, float *yaw);

/* Motion Driver(EMPL_TARGET_STM32F1) 平台时基接口（见 mpu_port.c） */
unsigned long get_tick_count(unsigned long *count);

/**
 * @brief 重置MPU6050的DMP的FIFO缓冲区
 *
 * 该函数用于清空MPU6050数字运动处理器（DMP）的FIFO（先进先出）缓冲区，以准备新的数据采集。
 */
void MPU6050_DMP_ResetFIFO(void);

/**
 * @brief 将四元数转换为欧拉角数组
 *
 * 该函数将输入的四元数转换为欧拉角（偏航角、俯仰角、滚转角），并将结果存储到一个浮点型数组中。
 *
 * @param quat 指向存储四元数的长整型数组的指针，数组长度应为4。
 * @param ypr 指向存储欧拉角结果的浮点型数组的指针，数组长度应为3，分别存储偏航角、俯仰角、滚转角。
 */
void quat_to_euler(long *quat, float *ypr);
/*
 * @brief 定义不同方向和安装方式下的方向矩阵
 * 
 * 这些方向矩阵用于表示不同的坐标轴朝向以及芯片安装方向与坐标的对应关系。
 * 每个方向矩阵都是一个 3x3 的矩阵，以一维数组的形式存储，按行优先顺序排列。
 * 这些矩阵可以用于将传感器测量的数据从芯片的本地坐标系转换到用户定义的全局坐标系。
 */
//static signed char gyro_orientation[9] = {
//    1, 0, 0,   // X轴正方向（默认）
//    0, 1, 0,   // Y轴正方向（默认）
//    0, 0, 1    // Z轴正方向（默认）
//};
// 默认Z轴向上配置
static signed char gyro_orientation[9] = {1,0,0, 0,1,0, 0,0,1};

/*
 * @brief X轴朝上时的方向矩阵
 * 
 * 该矩阵表示当X轴朝上时，芯片本地坐标系与全局坐标系之间的转换关系。
 * 矩阵的元素按行优先顺序存储，即 [row1_col1, row1_col2, row1_col3, row2_col1, ...]。
const signed char orientationMatrix_X_up[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1}; 
 * @brief Y轴朝上时的方向矩阵
 * 
 * 该矩阵表示当Y轴朝上时，芯片本地坐标系与全局坐标系之间的转换关系。
const signed char orientationMatrix_Y_up[9] = {1, 0, 0, 0, 0, 1, 0, -1, 0};
 * @brief Z轴朝上（默认）时的方向矩阵
 * 
 * 该矩阵表示当Z轴朝上时，芯片本地坐标系与全局坐标系之间的转换关系，通常作为默认设置。
const signed char orientationMatrix_Z_up[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
 * @brief 芯片正面朝上，丝印方向正常时的方向矩阵
 * 
 * 该矩阵表示当芯片正面朝上且丝印方向正常时，芯片本地坐标系与全局坐标系之间的转换关系。
const signed char orientationMatrix_chip_normal[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
 * @brief 芯片旋转180度时的方向矩阵
 * 
 * 该矩阵表示当芯片旋转180度时，芯片本地坐标系与全局坐标系之间的转换关系。
const signed char orientationMatrix_chip_rotated_180[9] = {-1, 0, 0, 0, -1, 0, 0, 0, 1};
 * @brief 芯片顺时针旋转90度时的方向矩阵
 * 
 * 该矩阵表示当芯片顺时针旋转90度时，芯片本地坐标系与全局坐标系之间的转换关系。
const signed char orientationMatrix_chip_rotated_cw_90[9] = {0, 1, 0, -1, 0, 0, 0, 0, 1};
 * @brief 芯片逆时针旋转90度时的方向矩阵
 * 
 * 该矩阵表示当芯片逆时针旋转90度时，芯片本地坐标系与全局坐标系之间的转换关系。
const signed char orientationMatrix_chip_rotated_ccw_90[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
*/
#endif
