/**
  * @file    bcm_door.c
  * @brief   BCM 车身总线应用层协议实现：门锁指令帧的封装与回执解析
  *
  *  该层不接触任何硬件/RTOS 对象，全部为纯函数：
  *    - 输出：ID + 数据场 + DLC
  *    - 输入：CAN 帧 -> 结构化回执
  *  这样做的目的：协议变更时只改本文件，且可用 PC 端脚本对照位域做回归。
  */
#include "bcm_door.h"
#include <stddef.h>      /* NULL（本层不依赖 STM32/stdint 以外的任何运行库） */

int BCM_Door_PackReq(const DoorLockReq* req, uint8_t seq, uint8_t sa, uint8_t da,
                     uint32_t* out_id, uint8_t* out_data, uint8_t* out_dlc)
{
    if (req == NULL || out_id == NULL || out_data == NULL || out_dlc == NULL) return -1;

    /* 参数合法性：任一项越界即拒绝封装，避免把非法帧推上总线 */
    if (req->door_mask == 0u || (req->door_mask & (uint8_t)~DOOR_MASK_ALL) != 0u) return -1;
    if (req->action != DOOR_ACT_UNLOCK && req->action != DOOR_ACT_LOCK) return -1;
    if (req->duty_pct > 100u) return -1;
    if (req->timeout_10ms == 0u) return -1;

    /* ID：目标地址(DH) + 源地址(SA) + 命令码(CMD) + 序号(SEQ) */
    *out_id   = BCM_ID_MAKE(BCM_PRI_SAFETY, da, sa, BCM_CMD_DOOR_LOCK, seq);

    /* 数据场：4 字节控制参数 */
    out_data[0] = req->door_mask;
    out_data[1] = req->action;
    out_data[2] = req->duty_pct;
    out_data[3] = req->timeout_10ms;
    *out_dlc    = DOOR_DLC_REQ;

    return 0;
}

int BCM_Door_ParseAck(uint32_t id, const uint8_t* data, uint8_t dlc,
                      uint8_t expect_da, DoorLockAck* out_ack)
{
    if (data == NULL || out_ack == NULL) return -1;

    /* 三重校验：命令码对 + 目标地址是本机 + DLC 正确，缺一不可 */
    if (BCM_ID_CMD(id) != BCM_CMD_DOOR_LOCK_ACK) return -1;
    if (BCM_ID_DA(id)  != expect_da)             return -1;
    if (dlc != DOOR_DLC_ACK)                     return -1;

    out_ack->req_seq    = data[0];
    out_ack->result     = data[1];
    out_ack->state_mask = data[2];

    return 0;
}
