/**
  * @file    bcm_task.c
  * @brief   车身控制任务：经 CAN 向车门控制器下发门锁指令并跟踪执行结果
  *
  *  职责（只做这一件事，且与其余任务解耦）：
  *   1. 接收其它任务投递的门锁请求（xDoorCmdQueue）；
  *   2. 调用协议层封装出"目标地址 + 控制参数"的 CAN 扩展帧，交给 bxCAN 发送；
  *   3. 等待车门控制器回执（按 SEQ 配对），成功则刷新全局门锁状态；
  *      超时则按上限重试，仍失败计入统一错误码；
  *   4. 观测总线健康（Bus-Off），异常时记账并触发恢复。
  *
  *  与其它任务的接口：
  *   - 请求入口：BCM_RequestDoorLock()（任务上下文，非阻塞）
  *   - 结果出口：g_state.door_lock_mask / door_cmd_busy（需持 xSystemStateMutex）
  */
#include "tasks.h"
#include "bsp_can.h"
#include "bcm_door.h"
#include "bsp_usart.h"
#include <stdio.h>

#define BCM_PERIOD_MS        (20)    /* 巡检周期；同时是接收阻塞上限 */
#define BCM_ACK_MARGIN_MS    (200)   /* 回执等待基准余量：帧传输(<1ms) + 从站处理 */
#define BCM_MAX_RETRY        (3)     /* 超时重发次数（首次发送不计入） */
#define BCM_TX_MAILBOX_MS    (20)    /* 等待发送邮箱空出的上限 */
#define BCM_BUSOFF_RECOVER_MS (1000) /* Bus-Off 强制重启 CAN 单元的最小间隔 */

#define DOOR_DUTY_DEFAULT    (70)    /* 闭锁电机驱动占空比(%) */
#define DOOR_TIMEOUT_DEFAULT (40)    /* 执行时限 40×10ms = 400ms */

    /* 在途请求状态机 */
static uint8_t     s_seq        = 0;
static uint8_t     s_busy       = 0;   /* 1 = 有请求在途（等回执/重试中） */
static uint8_t     s_retry      = 0;
static DoorLockReq s_req;
static uint8_t     s_req_seq;
static uint32_t    s_deadline_ms;

/* Bus-Off 记账：必须边沿触发。若按"电平"每拍记一次，总线持续关闭时
 * 会以 50 次/秒刷 Error_Record，16 条环形历史被灌满，其它错误的频次
 * 信息被彻底淹没；同样地，CAN 单元也不能每 20ms 重启一次。 */
static uint8_t  s_boff_latched    = 0;
static uint32_t s_boff_recover_ms = 0;

/* ------------------------------------------------------------------ */
/*  内部工具                                                            */
/* ------------------------------------------------------------------ */
static uint32_t BCM_Now(void)
{
    return (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/* 无符号差值判超时：天然规避 32 位节拍/毫秒计数器回绕 */
static uint8_t BCM_Expired(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0 ? 1u : 0u;
}

/* 回执等待窗口 = 从站执行时限 + 余量。
 * 关键一致性约束：回执语义为"执行完成"，因此窗口必须大于请求帧里的
 * 执行时限（timeout_10ms×10ms），否则每次都必然超时重发。 */
static uint32_t BCM_AckWindow(const DoorLockReq* r)
{
    return (uint32_t)r->timeout_10ms * 10u + BCM_ACK_MARGIN_MS;
}

static void BCM_Debug(const char* tag, uint8_t seq, uint8_t arg)
{
    char line[56];
    snprintf(line, sizeof(line), "BCM %s SEQ%u D%02X\r\n", tag, seq, arg);
    BSP_USART_SendString(DBG_USART, line);
}

static void BCM_SetBusy(uint8_t busy)
{
    if (xSystemStateMutex == NULL) return;
    if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
    {
        g_state.door_cmd_busy = busy;
        xSemaphoreGive(xSystemStateMutex);
    }
}

/* ------------------------------------------------------------------ */
/*  对外接口                                                            */
/* ------------------------------------------------------------------ */
int BCM_RequestDoorLock(uint8_t door_mask, uint8_t action)
{
    DoorLockReq req;

    if (xDoorCmdQueue == NULL) return -1;
    if (door_mask == 0u || (door_mask & (uint8_t)~DOOR_MASK_ALL) != 0u) return -1;
    if (action != DOOR_ACT_UNLOCK && action != DOOR_ACT_LOCK) return -1;

    req.door_mask    = door_mask;
    req.action       = action;
    req.duty_pct     = DOOR_DUTY_DEFAULT;
    req.timeout_10ms = DOOR_TIMEOUT_DEFAULT;

    /* 队列满即丢弃最新请求：门锁是幂等动作，不阻塞调用者、不积压过期指令 */
    if (xQueueSend(xDoorCmdQueue, &req, 0) != pdPASS) return -2;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  状态机：取请求 -> 发送 / 收回执 / 超时重发                              */
/* ------------------------------------------------------------------ */

/* 取一条新请求并发出；返回 1 表示已占据"在途" */
static uint8_t BCM_TryIssue(void)
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;

    if (s_busy || xDoorCmdQueue == NULL) return 0;
    if (xQueueReceive(xDoorCmdQueue, &s_req, 0) != pdPASS) return 0;

    s_seq     = (uint8_t)(s_seq + 1u);
    s_req_seq = s_seq;

    /* 目标地址 = 车门控制器；控制参数由数据场承载 */
    if (BCM_Door_PackReq(&s_req, s_req_seq, CAN_ADDR_BCM, CAN_ADDR_DOOR_CTRL,
                         &id, data, &dlc) != 0)
    {
        Error_Record(ERR_UNKNOWN);      /* 入口已校验，此处为兜底 */
        return 0;
    }

    if (BSP_CAN_Send(id, data, dlc, BCM_TX_MAILBOX_MS) != 0)
    {
        Error_Record(ERR_CAN_TX_TIMEOUT);
        return 0;
    }

    s_busy        = 1;
    s_retry       = 0;
    s_deadline_ms = BCM_Now() + BCM_AckWindow(&s_req);
    BCM_SetBusy(1);
    BCM_Debug("TX", s_req_seq, s_req.door_mask);
    return 1;
}

static void BCM_OnAck(const CanRxFrame* f)
{
    DoorLockAck ack;

    if (f == NULL) return;
    if (BCM_Door_ParseAck(f->id, f->data, f->dlc, CAN_ADDR_BCM, &ack) != 0) return;
    if (!s_busy || ack.req_seq != s_req_seq) return;   /* 非当前请求的回执：丢弃 */

    s_busy = 0;
    BCM_SetBusy(0);

    if (ack.result == DOOR_ACK_OK)
    {
        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                g_state.door_lock_mask = ack.state_mask;
                xSemaphoreGive(xSystemStateMutex);
            }
        }
        BCM_Debug("ACK", ack.req_seq, ack.state_mask);
    }
    else
    {
        Error_Record(ERR_CAN_TX_TIMEOUT);   /* 执行层失败按"未执行"处理 */
        BCM_Debug("NAK", ack.req_seq, ack.result);
    }
}

static void BCM_OnTimeout(void)
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;

    if (!s_busy) return;
    if (!BCM_Expired(BCM_Now(), s_deadline_ms)) return;

    if (s_retry < BCM_MAX_RETRY)
    {
        s_retry++;
        if (BCM_Door_PackReq(&s_req, s_req_seq, CAN_ADDR_BCM, CAN_ADDR_DOOR_CTRL,
                             &id, data, &dlc) == 0)
        {
            if (BSP_CAN_Send(id, data, dlc, BCM_TX_MAILBOX_MS) == 0)
            {
                s_deadline_ms = BCM_Now() + BCM_AckWindow(&s_req);
                BCM_Debug("RTX", s_req_seq, s_retry);
                return;
            }
        }
    }

    /* 重发到达上限：记账并放弃本条请求，释放总线给后续指令 */
    Error_Record(ERR_CAN_TX_TIMEOUT);
    s_busy = 0;
    BCM_SetBusy(0);
    BCM_Debug("FAIL", s_req_seq, s_retry);
}

/* Bus-Off 观测：边沿记账 + 节流恢复 */
static void BCM_PollBusOff(void)
{
    uint32_t now = BCM_Now();

    if (BSP_CAN_IsBusOff())
    {
        if (!s_boff_latched)
        {
            s_boff_latched    = 1;      /* 只在"进入"Bus-Off 的那一拍记账 */
            s_boff_recover_ms = now;
            Error_Record(ERR_CAN_BUSOFF);
        }
        /* ABOM 已使能，硬件通常会自行退出 Bus-Off；只有长时间不愈时才手工重启 */
        if ((uint32_t)(now - s_boff_recover_ms) >= BCM_BUSOFF_RECOVER_MS)
        {
            s_boff_recover_ms = now;
            BSP_CAN_RecoverBusOff();
        }
    }
    else
    {
        s_boff_latched = 0;             /* 已恢复，下次再进入仍会被记录 */
    }
}

/* ------------------------------------------------------------------ */
/*  任务主体                                                            */
/* ------------------------------------------------------------------ */
void BCM_Task(void* pvParameters)
{
    CanRxFrame f;

    (void)pvParameters;

    Health_Register(TASK_ID_BCM);

    /* 队列由 main() 在启动任务前创建，避免控制任务先于本任务调用请求入口 */
    BSP_CAN_Init();
    if (!BSP_CAN_IsReady()) Error_Record(ERR_CAN_INIT);

    for (;;)
    {
        Health_Report(TASK_ID_BCM);

        /* 1) 空闲则取一条请求发出 */
        BCM_TryIssue();

        /* 2) Bus-Off 观测（边沿记账 + 节流恢复，见 BCM_PollBusOff） */
        BCM_PollBusOff();

        /* 3) 收帧：阻塞上限即任务周期；帧到立即处理，无事件自然形成 20ms 节拍 */
        if (BSP_CAN_Receive(&f, BCM_PERIOD_MS))
        {
#if CAN_LOOPBACK_SELFTEST
            /* 自检模式：本节点发出的帧会被片内回环回来（SA == 本节点）。
             * 收到它即证明 组帧→发送邮箱→CAN控制器→接收FIFO→队列 整条链路通畅。 */
            if (BCM_ID_SA(f.id) == CAN_ADDR_BCM)
            {
                BCM_Debug("SELF-RX", BCM_ID_SEQ(f.id), BCM_ID_CMD(f.id));
            }
#endif
            BCM_OnAck(&f);
        }

        /* 4) 超时/重发检查 */
        BCM_OnTimeout();
    }
}
