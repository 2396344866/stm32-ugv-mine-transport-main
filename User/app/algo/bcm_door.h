#ifndef BCM_DOOR_H
#define BCM_DOOR_H

#include "stm32f10x.h"

/**
  * @brief  BCM 车身总线应用层协议：门锁指令
  *
  *  需求：车身控制模块（BCM）经 CAN 向车门控制器下发落锁指令，
  *        "目标地址"由 CAN 扩展 ID 承载，"控制参数"由数据场承载。
  *
  *  29 位扩展 ID 位域（本条总线统一使用扩展数据帧）：
  *    bit 28..26  PRI  优先级   0 最高（车身安全类取 1）
  *    bit 25..21  DA   目标地址 0..31 —— 指令要发给谁（车门控制器节点地址）
  *    bit 20..16  SA   源地址   0..31 —— 谁发的（BCM = 0x01）
  *    bit 15..8   CMD  命令码   0..255 —— 帧的语义
  *    bit  7..0   SEQ  报文序号 0..255 —— 请求/回执配对与去重
  *
  *  把 DA 放进 ID（而非数据场）的收益：bxCAN 过滤器可直接在验收阶段比对 DA，
  *  非本节点的报文根本不会进适配器 FIFO，也不产生中断，零 CPU 开销。
  */

/* ---- ID 位域宏 ---- */
#define BCM_ID_MAKE(pri, da, sa, cmd, seq)                       \
    ( (((uint32_t)(pri) & 0x07u) << 26) |                        \
      (((uint32_t)(da)  & 0x1Fu) << 21) |                        \
      (((uint32_t)(sa)  & 0x1Fu) << 16) |                        \
      (((uint32_t)(cmd) & 0xFFu) <<  8) |                        \
      (((uint32_t)(seq) & 0xFFu)      ) )

#define BCM_ID_PRIO(id)   ( (uint8_t)(((id) >> 26) & 0x07u) )
#define BCM_ID_DA(id)     ( (uint8_t)(((id) >> 21) & 0x1Fu) )
#define BCM_ID_SA(id)     ( (uint8_t)(((id) >> 16) & 0x1Fu) )
#define BCM_ID_CMD(id)    ( (uint8_t)(((id) >>  8) & 0xFFu) )
#define BCM_ID_SEQ(id)    ( (uint8_t)(((id)      ) & 0xFFu) )

#define BCM_ID_MASK_DA    ( (uint32_t)0x1Fu << 21 )
#define BCM_ID_MASK_SA    ( (uint32_t)0x1Fu << 16 )

/* ---- 优先级（PRI 位域，0 最高；车身总线只用到两档） ---- */
#define BCM_PRI_EMERG           (0u)   /* 保留：最高抢占 */
#define BCM_PRI_SAFETY          (1u)   /* 门锁/安全类指令 */

/* ---- 命令码 ---- */
#define BCM_CMD_DOOR_LOCK       (0x20u)  /* BCM -> 车门控制器：门锁执行请求 */
#define BCM_CMD_DOOR_LOCK_ACK   (0x21u)  /* 车门控制器 -> BCM：执行结果回执 */

/* ---- 门锁位图（数据场 byte0） ---- */
#define DOOR_MASK_FL            (0x01u)  /* 左前 */
#define DOOR_MASK_FR            (0x02u)  /* 右前 */
#define DOOR_MASK_RL            (0x04u)  /* 左后 */
#define DOOR_MASK_RR            (0x08u)  /* 右后 */
#define DOOR_MASK_ALL           (DOOR_MASK_FL | DOOR_MASK_FR | DOOR_MASK_RL | DOOR_MASK_RR)

/* ---- 执行动作（数据场 byte1） ---- */
#define DOOR_ACT_UNLOCK         (0x00u)  /* 解锁 */
#define DOOR_ACT_LOCK           (0x01u)  /* 落锁 */

/* ---- 数据长度 ---- */
#define DOOR_DLC_REQ            (4u)
#define DOOR_DLC_ACK            (3u)

/* ---- 回执结果（ACK 数据场 byte1） ---- */
#define DOOR_ACK_OK             (0x00u)
#define DOOR_ACK_NG             (0x01u)  /* 执行失败 */
#define DOOR_ACK_TIMEOUT        (0x02u)  /* 从站执行超时 */
#define DOOR_ACK_PARAM          (0x03u)  /* 参数非法 */

/* 门锁执行请求：BCM -> 车门控制器 */
typedef struct
{
    uint8_t door_mask;      /* bit0..3 对应 FL/FR/RL/RR，非 0 */
    uint8_t action;         /* DOOR_ACT_UNLOCK / DOOR_ACT_LOCK */
    uint8_t duty_pct;       /* 闭锁电机驱动占空比 0..100 (%)，典型 70 */
    uint8_t timeout_10ms;   /* 执行时限，单位 10ms（1..255 -> 10..2550ms） */
} DoorLockReq;

/* 门锁执行回执：车门控制器 -> BCM */
typedef struct
{
    uint8_t req_seq;        /* 被确认的请求帧 SEQ（配对依据） */
    uint8_t result;         /* DOOR_ACK_* */
    uint8_t state_mask;     /* 执行后门锁到位状态位图，位定义同 DOOR_MASK_* */
} DoorLockAck;

/**
  * @brief  封装"落锁请求"为 CAN 帧（纯函数，无硬件依赖，便于离线单测）
  * @param  req      门锁参数
  * @param  seq      报文序号
  * @param  sa       源地址（BCM）
  * @param  da       目标地址（车门控制器）
  * @retval 0 成功；-1 参数非法
  */
int BCM_Door_PackReq(const DoorLockReq* req, uint8_t seq, uint8_t sa, uint8_t da,
                     uint32_t* out_id, uint8_t* out_data, uint8_t* out_dlc);

/**
  * @brief  解析门锁回执帧
  * @param  id/ data/ dlc    收到的 CAN 帧
  * @param  expect_da        回执必须是发给本节点的（防止串帧）
  * @retval 0 成功；-1 非法
  */
int BCM_Door_ParseAck(uint32_t id, const uint8_t* data, uint8_t dlc,
                      uint8_t expect_da, DoorLockAck* out_ack);

#endif /* BCM_DOOR_H */
