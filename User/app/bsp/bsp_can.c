/**
  * @file    bsp_can.c
  * @brief   bxCAN 驱动：STM32F103RCT6 片内 CAN 控制器（CAN 2.0B）+ 外部 TJA1050
  *
  *  硬件链路：
  *    MCU PA11(CAN_RX) / PA12(CAN_TX)  --  TJA1050  --  CAN_H / CAN_L
  *  STM32 集成的是 bxCAN 控制器（协议层：位填充/CRC/ACK/错误界定/重传），
  *  只需外接 TJA1050 一类的收发器完成 TTL 逻辑电平 <-> CAN 差分电平转换。
  *
  *  关键设计：
  *   1) 位时基由 board_config.h 给出（PCLK1 36MHz, 9tq -> 500 kbit/s, 采样点 66.7%）；
  *   2) 接收走 FIFO0 挂号中断，ISR 内只做"取帧 + 入队"，解析交给任务上下文；
  *   3) 32 位掩码过滤器按"目标地址(DA)"硬件过滤，非本节点报文不产生中断开销；
  *   4) NART=0（开启自动重传）+ ABOM=1（自动退出 Bus-Off），总线错误由硬件自愈。
  */
#include "bsp_can.h"
#include "board_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

#define CAN_RX_QUEUE_LEN    (8)     /* 接收队列：8 帧 × sizeof(CanRxFrame) */

static QueueHandle_t s_rx_queue = NULL;
static uint8_t       s_ready    = 0;

/* 运行统计（供上层诊断与 Doc 中的验证依据） */
static uint32_t s_tx_ok   = 0;
static uint32_t s_rx_ok   = 0;
static uint32_t s_tx_fail = 0;
static uint32_t s_rx_drop = 0;

/* ------------------------------------------------------------------ */
/*  内部工具                                                            */
/* ------------------------------------------------------------------ */

/* 毫秒时标：依赖 FreeRTOS 节拍，仅任务上下文/ISR 可用 */
static uint32_t CAN_NowMs(void)
{
    return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/**
  * @brief  配置一个 32 位"标识符掩码模式"过滤器
  * @param  bank  过滤器组编号（0 起）
  * @param  id    期望的 29 位扩展 ID
  * @param  mask  关心位掩码（1=必须匹配）
  *
  *  32 位模式下 CAN_FxR1/R2 的布局：
  *    bit[31:21] = EXTID[28:18]  |  bit[20:3] = EXTID[17:0]
  *    bit[2]     = IDE           |  bit[1]    = RTR  |  bit[0] = 保留(0)
  *  因此 29 位 ID 需整体左移 3 位，再在低位补 IDE/RTR 控制位。
  *  掩码中加入 CAN_RTR_Remote(0x2) 而期望值用 CAN_RTR_Data(0x0)，
  *  即"必须是数据帧"；加入 CAN_Id_Extended(0x4) 即"必须是扩展帧"。
  */
static void CAN_FilterSetMask(uint8_t bank, uint32_t id, uint32_t mask)
{
    CAN_FilterInitTypeDef f;

    uint32_t id_reg   = ((id   << 3) & 0xFFFFFFF8u) | CAN_Id_Extended | CAN_RTR_Data;
    uint32_t mask_reg = ((mask << 3) & 0xFFFFFFF8u) | CAN_Id_Extended | CAN_RTR_Remote;

    f.CAN_FilterIdHigh        = (uint16_t)(id_reg   >> 16);
    f.CAN_FilterIdLow         = (uint16_t)(id_reg   & 0xFFFFu);
    f.CAN_FilterMaskIdHigh    = (uint16_t)(mask_reg >> 16);
    f.CAN_FilterMaskIdLow     = (uint16_t)(mask_reg & 0xFFFFu);
    f.CAN_FilterFIFOAssignment= CAN_Filter_FIFO0;
    f.CAN_FilterNumber        = bank;
    f.CAN_FilterMode          = CAN_FilterMode_IdMask;
    f.CAN_FilterScale         = CAN_FilterScale_32bit;
    f.CAN_FilterActivation    = ENABLE;
    CAN_FilterInit(&f);
}

/* ------------------------------------------------------------------ */
/*  初始化                                                              */
/* ------------------------------------------------------------------ */
void BSP_CAN_Init(void)
{
    GPIO_InitTypeDef     gpio;
    CAN_InitTypeDef      can;
    NVIC_InitTypeDef     nvic;
    uint8_t              ret;
    uint8_t              next_bank = 0;

    if (s_rx_queue == NULL)
    {
        s_rx_queue = xQueueCreate(CAN_RX_QUEUE_LEN, (UBaseType_t)sizeof(CanRxFrame));
    }

    /* --- 1. 时钟与引脚 ---
     * 使用默认映射 (CAN_RX=PA11 / CAN_TX=PA12)，无需 AFIO 重映射，
     * 因此不开 AFIO 时钟；RX 配置为上拉输入抑制浮空噪声，TX 为复用推挽。 */
    RCC_APB2PeriphClockCmd(BCM_CAN_GPIO_CLK, ENABLE);
    RCC_APB1PeriphClockCmd(BCM_CAN_CLK,      ENABLE);

    gpio.GPIO_Pin   = BCM_CAN_RX_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BCM_CAN_RX_PORT, &gpio);

    gpio.GPIO_Pin  = BCM_CAN_TX_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(BCM_CAN_TX_PORT, &gpio);

    /* --- 2. CAN 控制器参数 --- */
    CAN_DeInit(BCM_CAN);

    CAN_StructInit(&can);
    can.CAN_TTCM     = DISABLE;   /* 关闭时间触发通信模式 */
    can.CAN_ABOM     = ENABLE;    /* Bus-Off 由硬件自动恢复（128×11 隐性位） */
    can.CAN_AWUM     = ENABLE;    /* 报文来临时自动唤醒（配合低功耗） */
    can.CAN_NART     = DISABLE;   /* 关闭"禁止自动重传"= 允许自动重传，提升成功率 */
    can.CAN_RFLM     = DISABLE;   /* FIFO 溢出不锁定，新帧覆盖旧帧并由驱动计丢帧 */
    can.CAN_TXFP     = DISABLE;   /* 按 ID 优先级发送（车身指令优先级由 PRI 位域体现） */
#if CAN_LOOPBACK_SELFTEST
    can.CAN_Mode     = CAN_Mode_LoopBack;   /* 自发自收自检，无需对端节点 */
#else
    can.CAN_Mode     = CAN_Mode_Normal;
#endif
    can.CAN_SJW      = CAN_SJW_TQ;
    can.CAN_BS1      = CAN_BS1_TQ;
    can.CAN_BS2      = CAN_BS2_TQ;
    can.CAN_Prescaler= CAN_BRP;

    ret = CAN_Init(BCM_CAN, &can);
    if (ret == CAN_InitStatus_Failed)
    {
        s_ready = 0;
        return;                   /* 硬件无响应：不阻塞系统，由上层记错 */
    }

    /* --- 3. 过滤器 ---
     * bank0：只接收 目标地址(DA, bit25..21) == 本节点地址 的帧。
     *                其余位一律不关心 -> 硬件级过滤，非本节点报文不进 FIFO。 */
    CAN_FilterSetMask(next_bank++,
                      ((uint32_t)CAN_ADDR_BCM << 21),
                      ((uint32_t)0x1Fu        << 21));

#if CAN_LOOPBACK_SELFTEST
    /* bank1：自检模式下补一条 SA==本节点 的过滤，用于回收自己发出的帧 */
    CAN_FilterSetMask(next_bank++,
                      ((uint32_t)CAN_ADDR_BCM << 16),
                      ((uint32_t)0x1Fu        << 16));
#endif

    /* --- 4. FIFO0 挂号中断 --- */
    CAN_ITConfig(BCM_CAN, CAN_IT_FMP0, ENABLE);

    nvic.NVIC_IRQChannel                   = BCM_CAN_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = BCM_CAN_RX0_IRQ_PREEMPT;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    s_ready = 1;
}

uint8_t BSP_CAN_IsReady(void) { return s_ready; }

/* ------------------------------------------------------------------ */
/*  发送                                                                */
/* ------------------------------------------------------------------ */
int BSP_CAN_Send(uint32_t ext_id, const uint8_t* data, uint8_t dlc, uint32_t timeout_ms)
{
    CanTxMsg  tx;
    uint8_t   i;
    uint32_t  start;

    if (!s_ready) return -1;
    if (CAN_GetFlagStatus(BCM_CAN, CAN_FLAG_BOF) == SET) return -3;

    tx.StdId = 0u;
    tx.ExtId = ext_id & 0x1FFFFFFFu;
    tx.IDE   = CAN_Id_Extended;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = (dlc > 8u) ? 8u : dlc;
    for (i = 0; i < 8u; i++)
    {
        tx.Data[i] = ((i < tx.DLC) && (data != NULL)) ? data[i] : 0u;
    }

    /* 等待空闲邮箱。进入邮箱后由硬件自动仲裁/重传直至成功（NART=0），
     * 故此处不等 RQCP 完成——避免在离线总线上长时间占住任务。 */
    start = CAN_NowMs();
    for (;;)
    {
        if (CAN_Transmit(BCM_CAN, &tx) != CAN_NO_MB)
        {
            s_tx_ok++;
            return 0;
        }
        if ((uint32_t)(CAN_NowMs() - start) >= timeout_ms)
        {
            s_tx_fail++;
            return -2;
        }
        vTaskDelay((TickType_t)1);
    }
}

/* ------------------------------------------------------------------ */
/*  接收                                                                */
/* ------------------------------------------------------------------ */
int BSP_CAN_Receive(CanRxFrame* frame, uint32_t timeout_ms)
{
    if (frame == NULL || s_rx_queue == NULL) return 0;

    if (xQueueReceive(s_rx_queue, frame, pdMS_TO_TICKS(timeout_ms)) == pdPASS)
        return 1;
    return 0;
}

void BSP_CAN_RxFifo0ISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (s_rx_queue == NULL) return;

    while (CAN_MessagePending(BCM_CAN, CAN_FIFO0) > 0)
    {
        CanRxMsg     rx;
        CanRxFrame   f;
        uint8_t      i;

        if (CAN_GetFlagStatus(BCM_CAN, CAN_FLAG_FOV0) == SET)
        {
            CAN_ClearFlag(BCM_CAN, CAN_FLAG_FOV0);   /* FIFO 溢出：旧帧被覆盖 */
            s_rx_drop++;
        }

        CAN_Receive(BCM_CAN, CAN_FIFO0, &rx);

        f.id    = (rx.IDE == CAN_Id_Extended) ? rx.ExtId : (uint32_t)rx.StdId;
        f.dlc   = rx.DLC;
        f.rx_ms = CAN_NowMs();
        for (i = 0; i < 8u; i++) f.data[i] = (i < rx.DLC) ? rx.Data[i] : 0u;

        if (xQueueSendFromISR(s_rx_queue, &f, &xHigherPriorityTaskWoken) == pdPASS)
            s_rx_ok++;
        else
            s_rx_drop++;          /* 任务未及时消费，丢弃但计数，供上层观测 */
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ------------------------------------------------------------------ */
/*  总线诊断                                                            */
/* ------------------------------------------------------------------ */
uint8_t BSP_CAN_IsBusOff(void)
{
    return (CAN_GetFlagStatus(BCM_CAN, CAN_FLAG_BOF) == SET) ? 1u : 0u;
}

void BSP_CAN_RecoverBusOff(void)
{
    if (!BSP_CAN_IsBusOff()) return;

    /* ABOM 已使能，硬件会自动退出 Bus-Off；本函数用于长期离线、
     * 重新挂接收发器等场景强制重启 CAN 单元（完整走一遍 Init）。 */
    CAN_ITConfig(BCM_CAN, CAN_IT_FMP0, DISABLE);
    BSP_CAN_Init();
}

void BSP_CAN_GetCounters(uint32_t* tx_ok, uint32_t* rx_ok, uint32_t* tx_fail, uint32_t* rx_drop)
{
    if (tx_ok)    *tx_ok    = s_tx_ok;
    if (rx_ok)    *rx_ok    = s_rx_ok;
    if (tx_fail)  *tx_fail  = s_tx_fail;
    if (rx_drop)  *rx_drop  = s_rx_drop;
}
