#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32f10x.h"
#include "FreeRTOS.h"

/**
  * @brief  bxCAN 驱动（STM32F103RCT6 片内 CAN 控制器 + TJA1050 收发器）
  *
  *  数据流：
  *    发送——任务上下文调用 BSP_CAN_Send()，写入发送邮箱并等待硬件完成；
  *    接收——FIFO0 挂号中断在 ISR 内把帧搬进 FreeRTOS 队列，
  *           任务上下文用 BSP_CAN_Receive() 取帧，符合中断底半部原则。
  *
  *  注意：收发 API 依赖节拍，仅可在调度器启动后（任务上下文）调用。
  */

/* 接收帧（句柄化：屏蔽 StdPeriph 的 CanRxMsg 结构，任务层不接触库类型） */
typedef struct
{
    uint32_t id;        /* 29 位扩展 ID */
    uint8_t  dlc;       /* 数据长度 0..8 */
    uint8_t  data[8];
    uint32_t rx_ms;     /* 接收时标(ms) */
} CanRxFrame;

/* 初始化：GPIO(PA11/PA12) + 时钟 + 波特率 + 过滤器 + FIFO0 中断
 * 失败时 s_ready 保持 0，所有收发调用返回错误（不阻塞系统） */
void    BSP_CAN_Init(void);
uint8_t BSP_CAN_IsReady(void);

/**
  * @brief  发送一帧扩展数据帧
  * @param  ext_id      29 位扩展标识符
  * @param  data        数据场首地址（可为 NULL 时 dlc 必须为 0）
  * @param  dlc         数据长度 0..8
  * @param  timeout_ms  等待邮箱空出的最长时间
  * @retval  0 成功; -1 未就绪; -2 邮箱忙超时; -3 总线关闭(Bus-Off)
  */
int     BSP_CAN_Send(uint32_t ext_id, const uint8_t* data, uint8_t dlc, uint32_t timeout_ms);

/* 任务上下文取一帧；timeout_ms 为阻塞上限；返回 1 取到帧，0 超时 */
int     BSP_CAN_Receive(CanRxFrame* frame, uint32_t timeout_ms);

/* 总线诊断与恢复（ABOM 已使能，硬件可自行退出 Bus-Off；此处供上层观测/强制恢复） */
uint8_t BSP_CAN_IsBusOff(void);
void    BSP_CAN_RecoverBusOff(void);
void    BSP_CAN_GetCounters(uint32_t* tx_ok, uint32_t* rx_ok, uint32_t* tx_fail, uint32_t* rx_drop);

/* 由 stm32f10x_it.c 在 USB_LP_CAN1_RX0_IRQHandler 中调用 */
void    BSP_CAN_RxFifo0ISR(void);

#endif /* BSP_CAN_H */
