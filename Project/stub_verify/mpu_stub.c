/**
  * @file    mpu_stub.c
  * @brief   仅用于本机编译/链接验证的 MPU6050 DMP 桩函数（不属于正式源码）
  *
  *  背景：stm32-ugv-mine-transport 仓库按第三方许可合规要求排除了
  *  Motion_driver eMPL 源码（inv_mpu.c/h 等），故 User/Modules/mpu6050/MPU6050.c
  *  在本机无法编译。为了让新增的 CAN 源码能通过完整编译 + 链接验证，
  *  用本桩文件替换 MPU6050.c，提供被 main.c / control_task.c 引用的 5 个符号。
  */
#include "stm32f10x.h"

void     MPU6050_Init(void)                         { }
uint8_t  MPU6050_GetID(void)                        { return 0x68u; }
void     MPU6050_DMP_Init(void)                     { }
int      MPU6050_DMP_GetData(float* pitch, float* roll, float* yaw)
{
    if (pitch) *pitch = 0.0f;
    if (roll)  *roll  = 0.0f;
    if (yaw)   *yaw   = 0.0f;
    return 0;
}
