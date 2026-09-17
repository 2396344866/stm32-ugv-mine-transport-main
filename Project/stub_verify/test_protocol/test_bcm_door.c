/**
  * @file    test_bcm_door.c
  * @brief   协议层主机单元测试（PC 上真实运行，不依赖硬件）
  *
  *  验证目标：把文档 Doc/CAN_BCM车门落锁.md 里写死的报文实例，
  *  反过来拴住代码——文档与实现任何一个被改错，这里就会失败。
  *
  *  做法：bcm_door.h 只依赖整型宽度别名，用本目录下的 stm32f10x.h 替身头文件
  *  即可在 PC 上编译运行，直接调用真实函数检查输入输出。
  */
#include "bcm_door.h"
#include <stdio.h>
#include <string.h>

static int s_fail = 0;

/* 节点地址：必须与 User/board_config.h 中的 CAN_ADDR_BCM / CAN_ADDR_DOOR_CTRL
 * 保持一致。此处独立定义，是为了让测试脱离 STM32 设备头文件在 PC 上运行。 */
#define T_ADDR_BCM           (0x01u)
#define T_ADDR_DOOR_CTRL     (0x10u)

#define CHECK(cond, msg)                                        \
    do {                                                        \
        if (cond) { printf("  [ok]   %s\n", msg); }              \
        else      { printf("  [FAIL] %s\n", msg); s_fail++; }    \
    } while (0)

/* ID 位域：封装 -> 回拆，必须与文档 §3.3 的实例帧逐字段一致 */
static void test_id_fields(void)
{
    uint32_t id = BCM_ID_MAKE(BCM_PRI_SAFETY, T_ADDR_DOOR_CTRL,
                              T_ADDR_BCM, BCM_CMD_DOOR_LOCK, 0x07u);

    printf("[ID 位域]\n");
    CHECK(id == 0x06012007u,                 "落锁帧 ID == 0x0601 2007");
    CHECK(BCM_ID_PRIO(id) == 1u,             "PRI  == 1");
    CHECK(BCM_ID_DA(id)   == 0x10u,          "DA   == 0x10 (车门控制器)");
    CHECK(BCM_ID_SA(id)   == 0x01u,          "SA   == 0x01 (BCM)");
    CHECK(BCM_ID_CMD(id)  == 0x20u,          "CMD  == 0x20");
    CHECK(BCM_ID_SEQ(id)  == 0x07u,          "SEQ  == 0x07");

    uint32_t ack = BCM_ID_MAKE(BCM_PRI_SAFETY, T_ADDR_BCM,
                               T_ADDR_DOOR_CTRL, BCM_CMD_DOOR_LOCK_ACK, 0x07u);
    CHECK(ack == 0x04302107u,                "回执帧 ID == 0x0430 2107");
    CHECK(BCM_ID_DA(ack) == T_ADDR_BCM, "回执帧 DA 指向 BCM");
}

/* 请求封装：数据场 4 字节控制参数 */
static void test_pack_req(void)
{
    DoorLockReq req;
    uint32_t id = 0;
    uint8_t  data[8];
    uint8_t  dlc = 0;
    int      ret;

    printf("[请求封装]\n");
    req.door_mask    = 0x0Fu;      /* 四门 */
    req.action       = DOOR_ACT_LOCK;
    req.duty_pct     = 70u;
    req.timeout_10ms = 40u;        /* 400ms */

    ret = BCM_Door_PackReq(&req, 0x07u, T_ADDR_BCM,
                           T_ADDR_DOOR_CTRL, &id, data, &dlc);
    CHECK(ret == 0,                              "合法参数 -> 返回 0");
    CHECK(dlc == DOOR_DLC_REQ,                   "DLC == 4");
    CHECK(id  == 0x06012007u,                    "ID  == 0x0601 2007");
    CHECK(data[0] == 0x0Fu && data[1] == 0x01u &&
          data[2] == 0x46u && data[3] == 0x28u,  "Data == 0F 01 46 28");

    /* 非法参数必须被挡住，不能推上总线 */
    DoorLockReq bad = req;
    bad.door_mask = 0x00u;
    CHECK(BCM_Door_PackReq(&bad, 0x07u, T_ADDR_BCM,
                           T_ADDR_DOOR_CTRL, &id, data, &dlc) == -1,
          "door_mask=0 -> 拒绝");

    bad = req; bad.door_mask = 0x30u;
    CHECK(BCM_Door_PackReq(&bad, 0x07u, T_ADDR_BCM,
                           T_ADDR_DOOR_CTRL, &id, data, &dlc) == -1,
          "door_mask 越界 -> 拒绝");

    bad = req; bad.duty_pct = 101u;
    CHECK(BCM_Door_PackReq(&bad, 0x07u, T_ADDR_BCM,
                           T_ADDR_DOOR_CTRL, &id, data, &dlc) == -1,
          "duty>100 -> 拒绝");

    bad = req; bad.timeout_10ms = 0u;
    CHECK(BCM_Door_PackReq(&bad, 0x07u, T_ADDR_BCM,
                           T_ADDR_DOOR_CTRL, &id, data, &dlc) == -1,
          "timeout=0 -> 拒绝");

    CHECK(BCM_Door_PackReq(NULL, 0x07u, T_ADDR_BCM,
                           T_ADDR_DOOR_CTRL, &id, data, &dlc) == -1,
          "空指针 -> 拒绝");
}

/* 回执解析：含"发给别人"的串帧防御 */
static void test_parse_ack(void)
{
    DoorLockAck ack;
    uint8_t     data[3] = {0x07u, DOOR_ACK_OK, 0x0Fu};
    uint32_t    id = BCM_ID_MAKE(BCM_PRI_SAFETY, T_ADDR_BCM,
                                 T_ADDR_DOOR_CTRL,
                                 BCM_CMD_DOOR_LOCK_ACK, 0x07u);

    printf("[回执解析]\n");
    CHECK(BCM_Door_ParseAck(id, data, 3u, T_ADDR_BCM, &ack) == 0,
          "正常回执 -> 返回 0");
    CHECK(ack.req_seq == 0x07u && ack.result == DOOR_ACK_OK &&
          ack.state_mask == 0x0Fu, "字段解析正确 (seq/result/mask)");

    /* 以下四类是实际会吃到的脏数据 */
    CHECK(BCM_Door_ParseAck(BCM_ID_MAKE(BCM_PRI_SAFETY, T_ADDR_BCM,
                            T_ADDR_DOOR_CTRL, BCM_CMD_DOOR_LOCK, 0x07u),
                            data, 3u, T_ADDR_BCM, &ack) == -1,
          "CMD 不是回执 -> 拒绝");
    CHECK(BCM_Door_ParseAck(BCM_ID_MAKE(BCM_PRI_SAFETY, 0x02u,
                            T_ADDR_DOOR_CTRL, BCM_CMD_DOOR_LOCK_ACK, 0x07u),
                            data, 3u, T_ADDR_BCM, &ack) == -1,
          "发给别的节点 -> 拒绝");
    CHECK(BCM_Door_ParseAck(id, data, 2u, T_ADDR_BCM, &ack) == -1,
          "DLC 不符 -> 拒绝");
    CHECK(BCM_Door_ParseAck(id, NULL, 3u, T_ADDR_BCM, &ack) == -1,
          "空数据指针 -> 拒绝");
}

int main(void)
{
    test_id_fields();
    test_pack_req();
    test_parse_ack();

    printf("\n%s (失败 %d 项)\n", s_fail ? "FAIL" : "ALL PASS", s_fail);
    return s_fail ? 1 : 0;
}
