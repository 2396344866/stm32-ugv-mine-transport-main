/**
  * @file    bsp_uwb.c
  * @brief   UWB 定位子系统主机侧驱动（BU03 模块 UART-AT）
  *
  *  说明：BU03 为基于 DW3000 的 UWB 模组（板载 RF/电源管理/协处理器），本工程经 UART-AT 驱动（模块内置 AT 固件），主机 STM32F103RCT6 以 AT 指令配置测距并读取结果。
  *        具体 AT 指令集随模块固件版本而定；此处给出通用框架：
  *        启动测距指令 + 距离结果解析（匹配 "DIST:x.xxx" 文本）。
  */
#include "bsp_uwb.h"
#include "bsp_usart.h"
#include "board_config.h"
#include <string.h>
#include <stdio.h>

void BSP_UWB_Init(void)
{
    BSP_USART_Init(UWB_USART, UWB_USART_BAUD, 1);  /* 部分重映射到 PC10/PC11 */
}

int UWB_StartRanging(void)
{
    /* 通用启动测距指令（按模块固件调整） */
    BSP_USART_SendString(UWB_USART, "AT+KW=RANGE,1\r\n");
    return 0;
}

int UWB_ReadDistance(float* dist_m)
{
    uint8_t buf[96];
    int n = BSP_USART_Read(UWB_USART, buf, sizeof(buf) - 1, 120);
    if (n <= 0) return -1;
    buf[n] = 0;

    /* 解析 "DIST:x.xxx"（模块输出距离，单位米） */
    char* p = strstr((char*)buf, "DIST:");
    if (p && (sscanf(p, "DIST:%f", dist_m) == 1))
    {
        return 0;
    }
    /* 回退：扫描首个浮点数（兼容不同固件文本） */
    float v = 0.0f;
    if (sscanf((char*)buf, "%f", &v) == 1)
    {
        *dist_m = v;
        return 0;
    }
    return -1;
}
