# CAN 车身总线：BCM → 车门控制器「落锁指令」实现

> 目标：车身控制模块（**BCM**）经 CAN 向车门控制器下发落锁指令，
> **目标地址**由 CAN 扩展 ID 承载，**控制参数**由数据场承载。
> 芯片：STM32F103RCT6（片内 **bxCAN**，CAN 2.0A/B）+ 外部收发器 **TJA1050**。

---

## 0. 结论先行

| 项 | 取值 |
|---|---|
| 可行性 | STM32F103RCT6 自带 bxCAN 控制器（协议层），片外只需 1 颗 TJA1050 做电平转换 |
| 引脚 | CAN_RX=PA11、CAN_TX=PA12（**默认映射**，不做 AFIO 重映射） |
| 速率 | 500 kbit/s（PCLK1 36MHz，BRP=8，1+5+3=9tq，采样点 66.7%） |
| 帧类型 | 扩展数据帧（IDE=1），29 位 ID |
| 落锁帧 | ID `0x0601 2007`　data `0F 01 46 28`　DLC=4 |
| 回执帧 | ID `0x0430 2107`　data `07 00 0F`　DLC=3 |
| 单帧耗时 | 99 bit → **198 µs**（含最坏位填充约 248 µs） |
| 新增源码 | `bsp_can.c/h`、`bcm_door.c/h`、`bcm_task.c` |

---

## 1. 硬件链路

```
STM32F103RCT6            TJA1050              CAN 总线
  PA12 (CAN_TX) ──TXD──▶  driver  ──CAN_H──┐
  PA11 (CAN_RX) ◀──RXD──  receive ◀─CAN_L──┘
                          S(静音)=GND
                          Vref / SPLIT  悬空
```

| 关注点 | 说明 |
|---|---|
| 谁来干协议层的活 | MCU 片内 bxCAN：位填充、CRC15、**ACK 应答**、错误计数、**自动重传**、Bus-Off 管理 |
| TJA1050 只干什么 | TTL 逻辑电平 ↔ CAN 差分电平（CAN_H/CAN_L），不做协议处理 |
| 终端电阻 | 总线两端各 1 只 **120 Ω**（F103 板载通常无，需外部补或短接 P1 跳线），否则小负载下波形振铃、ACK 错误 |
| 为何不使用 PB8/PB9 | PB8/PB9 是 CAN **完全重映射**脚，已被 OLED 软件 I2C 占用；默认映射 PA11/PA12 与 USB 复用，本工程未用 USB，无冲突 |
| AFIO 时钟 | 默认映射不需 AFIO 重映射寄存器，**不开** `AFIOEN`；只有重映射时才需要 |

---

## 2. 位时基（决定波特率）

`User/board_config.h`：`CAN_BRP / CAN_BS1_TQ / CAN_BS2_TQ / CAN_SJW_TQ`

> ⚠️ 位段宏刻意加了 `_TQ` 后缀：`CAN_BS1 / CAN_BS2 / CAN_SJW` 同时是
> `CAN_InitTypeDef` 的成员名，若用它们做宏，`can.CAN_BS1 = CAN_BS1;`
> 会被展开成 `(...).((uint8_t)0x04)` 直接编译报错。

```
tq        = BRP / PCLK1           = 8 / 36MHz ≈ 0.2222 µs
位时间    = 1(SYNC) + BS1 + BS2   = 1 + 5 + 3 = 9 tq = 2.0 µs
波特率    = 1 / 2.0µs             = 500 kbit/s
采样点    = (1+BS1) / 9           = 66.7%
重同步跳转宽度 SJW = 1 tq
```

| 目标速率 | BRP | 位段（1+BS1+BS2） | 采样点 |
|---|---|---|---|
| **500 kbit/s（本工程）** | 8 | 1+5+3 = 9 tq | 66.7% |
| 250 kbit/s | 16 | 9 tq | 66.7% |
| 125 kbit/s | 32 | 9 tq | 66.7% |

> 只需改 `CAN_BRP` 一处；位段不动是为了保持采样点不变，
> 换速率时不会引入新的采样点偏移风险。

---

## 3. 报文格式（本节是核心）

### 3.1 扩展 ID 位域——承载「目标地址」

| 位域 | 位 | 含义 | 落锁帧取值 |
|---|---|---|---|
| PRI | 28..26 | 优先级，**0 最高**；车身门锁属安全类，取 1 | `1` |
| **DA** | 25..21 | **目标地址**：指令发给谁 | `0x10` 车门控制器 |
| SA | 20..16 | 源地址：谁发的 | `0x01` BCM |
| CMD | 15..8 | 命令码：帧的语义 | `0x20` DOOR_LOCK |
| SEQ | 7..0 | 报文序号：请求/回执配对与去重（每帧 +1） | 自增 |

```
29 28 27 26 25 ... 21 20 ... 16 15 ... 8  7 ... 0
|  PRI  |    DA    |    SA    |   CMD   |  SEQ  |
```

### 3.2 数据场——承载「控制参数」（请求帧，`CMD=0x20`，DLC=4）

| 字节 | 参数 | 说明 | 示例 |
|---|---|---|---|
| D0 | `door_mask` | 门锁位图：bit0 左前 / bit1 右前 / bit2 左后 / bit3 右后 | `0x0F` 四门全选 |
| D1 | `action` | `0x00` 解锁 / `0x01` 落锁 | `0x01` |
| D2 | `duty_pct` | 闭锁电机驱动占空比 0..100 (%) | `0x46` = 70% |
| D3 | `timeout_10ms` | 执行时限，单位 10 ms（1..255 → 10..2550 ms） | `0x28` = 400 ms |

回执帧（`CMD=0x21`，DLC=3）：

| 字节 | 参数 | 说明 |
|---|---|---|
| D0 | `req_seq` | 被确认的请求帧 SEQ |
| D1 | `result` | `0x00` 成功 / `0x01` 执行失败 / `0x02` 从站超时 / `0x03` 参数非法 |
| D2 | `state_mask` | 执行后门锁到位状态，位定义同 `door_mask` |

### 3.3 实例帧（可直接对照抓包阅读）

**① 全车落锁（BCM → 车门控制器）**

| 字段 | 值 | 拆解 |
|---|---|---|
| ID | `0x0601 2007` | PRI=1, DA=0x10, SA=0x01, CMD=0x20, SEQ=0x07 |
| DLC | 4 | |
| Data | `0F 01 46 28` | 四门 / 落锁 / 70% / 400 ms |

**② 仅左前门解锁**

| 字段 | 值 |
|---|---|
| ID | `0x0601 2008` | SEQ=0x08（上一条 +1） |
| Data | `01 00 46 28` | 左前 / 解锁 / 70% / 400 ms |

**③ 回执（车门控制器 → BCM）**

| 字段 | 值 | 拆解 |
|---|---|---|
| ID | `0x0430 2107` | PRI=1, **DA=0x01(BCM)**, SA=0x10, CMD=0x21, SEQ=0x07 |
| Data | `07 00 0F` | 确认 SEQ=7 / 成功 / 四门到位 |

### 3.4 为什么把「目标地址」放进 ID 而不是数据场

放进 ID 后可由 **bxCAN 验收过滤器**在硬件层比对 DA：
非本节点的报文——**不进 FIFO、不触发中断、不占用 CPU**。
若把地址塞进数据场，所有节点都必须先收下全部报文再软件判地址，白白付出中断与拷贝开销。

驱动中的过滤器（32 位标识符掩码模式，`bsp_can.c`）：

```
关心位掩码 = DA 字段 (0x1F << 21)  |  IDE=扩展  |  RTR=数据帧
期望值     = 本节点地址 << 21
```

---

## 4. 时序预算

| 环节 | 数量级 | 依据 |
|---|---|---|
| 扩展数据帧长度 | `67 + 8×DLC = 99 bit` | SOF/仲裁/控制/DLC/CRC/ACK/EOF/IFS 之和 |
| 单帧传输时间 | 99 × 2 µs = **198 µs**（最坏位填充约 248 µs） | @500 kbit/s |
| BCM 任务节拍 | **20 ms**（阻塞接收超时 = 周期） | 远大于单帧时间，帧不会积压 |
| 回执等待窗口 | `timeout_10ms×10 + 200 ms` = **600 ms** | 从站执行时限 + 帧传输/处理余量 |
| 超时重发 | 上限 **3 次**，之后记错并放弃本条 | 避免离线总线被永久占住 |

> **一致性约束（易错点）**：回执语义是「执行完成」，因此回执窗口
> **必须大于**请求帧里下发的执行时限，否则每条指令都必然超时重发。
> 代码里由 `BCM_AckWindow()` 从请求参数现算，不写死常量。

---

## 5. 软件分层与新增文件

| 层 | 文件 | 职责 |
|---|---|---|
| **BSP** | `User/app/bsp/bsp_can.c/h` | GPIO/时钟/波特率/**过滤器**/发送邮箱/FIFO0 中断 → 队列/Bus-Off |
| **协议** | `User/app/algo/bcm_door.c/h` | ID 位域宏、请求封装、回执解析、参数校验（**纯函数**，不碰硬件） |
| **任务** | `User/app/tasks/bcm_task.c` | 取请求 → 封装 → 发送 → 等回执 → 超时重发 → 写全局状态 |

关键实现选择：

| 选择 | 理由 |
|---|---|
| 接收走 **FIFO0 中断 + 队列**，解析放任务上下文 | 符合中断底半部原则，ISR 内只做「取帧 + 入队」 |
| `NART=0`（开自动重传）、`ABOM=1`（自动退出 Bus-Off） | 总线错误由硬件自愈，软件不参与位级仲裁 |
| `TXFP=0`（按 ID 优先级发送） | 使 ID 里的 PRI 位域真正起作用 |
| 发送只等**邮箱空**，不等 RQCP 完成 | 避免在离线总线上长期占住任务；重发时机交给硬件 |
| NVIC 抢占优先级 **11**（= `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`） | 优先级数值 ≥ 11 才能安全调用 `xQueueSendFromISR` |
| 协议层无 RTOS/硬件依赖 | 便于离线对照位域做回归 |

---

## 6. 落锁指令执行流程

```
其他任务 ──BCM_RequestDoorLock(mask, action)──▶ xDoorCmdQueue(4)
                                                    │
BCM 任务 20ms 巡检 ──取请求──▶ bcm_door 封装(ID+data+DLC) ──▶ BSP_CAN_Send
                                                              │ 进发送邮箱
                                              bxCAN 自动仲裁/重传 ──▶ 总线
                                                              │
                                    FIFO0 中断 ──▶ rx 队列 ──▶ BCM 任务解析回执
                                          配对 SEQ ──▶ 成功：刷新 g_state.door_lock_mask
                                                      超时：重发（≤3 次）→ 仍失败记 ERR_CAN_TX_TIMEOUT
```

状态出口（持 `xSystemStateMutex` 访问）：

| 字段 | 含义 |
|---|---|
| `g_state.door_lock_mask` | 最近一次成功回执回报的门锁到位位图 |
| `g_state.door_cmd_busy` | 有在途指令（等待回执/重试中） |

OLED 新增第 5 页（`HMI_PAGES` 由 4 改 5，KEY1 切换）用于就地查看这两个字段：

```
== DOOR/CAN ==
Lock: 0110        <- bit0..3 = 左前/右前/左后/右后 已锁置 1
Busy: 0           <- 1 = 有指令在途（等回执/重试中）
```

### 6.1 触发源：安全状态机联动

控制任务进入 `SAFE_BRAKE` / `SAFE_ERROR`（越限锁定）的**那一拍**，
自动请求一次全车落锁——这与井下运输车的实际安全逻辑一致：
车辆已被锁定停车时，车门必须处于落锁状态。

```c
/* control_task.c，仅在"刚进入"锁定态时触发一次 */
if ((st == SAFE_BRAKE || st == SAFE_ERROR) && s_prev_safe != st)
    BCM_RequestDoorLock(DOOR_MASK_ALL, DOOR_ACT_LOCK);
```

> 判定用「状态跳变」而非「状态本身」，否则 20ms 控制环会在锁定期间
> 每拍下发一条指令，直接灌满请求队列。

调试串口（USART2）事件行：`BCM TX SEQ<n> D<mask>` / `ACK` / `NAK` / `RTX`（重发）/ `FAIL`。

---

## 7. 对既有工程的改动

| 文件 | 改动 |
|---|---|
| `User/board_config.h` | 新增 `USE_CAN_SUBSYS=1`、CAN 引脚/时钟/IRQ、位时基常量、节点地址、`CAN_LOOPBACK_SELFTEST` 开关 |
| `User/app/sys/error.h|error.c` | 新增 `ERR_CAN_TX_TIMEOUT / ERR_CAN_BUSOFF / ERR_CAN_INIT`；计数数组改为 `ERR_CODE_COUNT` 统一边界 |
| `User/app/tasks/tasks.h` | 新增 `TASK_ID_BCM`、`TASK_PRIO_BCM(3)`、`TASK_STACK_BCM(256)`、`xDoorCmdQueue`、`BCM_RequestDoorLock()`、`SystemState.door_lock_mask / door_cmd_busy` |
| `User/main.c` | `#if USE_CAN_SUBSYS` 创建门锁请求队列 `xDoorCmdQueue` + BCM 任务（**队列必须先于任务创建**：控制任务可能在 BCM 任务起来之前就调用请求入口） |
| `User/app/tasks/control_task.c` | 安全状态机「刚进入」BRAKE/ERROR 时联动请求全车落锁（见 §6.1） |
| `User/app/tasks/hmi_task.c` | `HMI_PAGES` 4→5，新增 Page4 显示门锁位图与在途标志 |
| `User/stm32f10x_it.c` | 新增 `USB_LP_CAN1_RX0_IRQHandler` → `BSP_CAN_RxFifo0ISR()` |
| `Project/build.py` | 源文件清单加入 3 个新 `.c` |
| `Project/stm32f103RCT6.uvprojx` | App_BSP / App_Algo / App_Tasks 三组各登记 1 个新文件（已 XML 校验） |
| `Project/stub_verify/` | 本机编译验证所需目录（`build_verify.py` + `mpu_stub.c`，见 §8.1） |

---

## 8. 验证方法

### 8.1 编译与链接（本机已通过）

见 §8.3 结果表。一条命令串起三项检查：

```bash
cd Project/stub_verify && python build_verify.py
```

1. **固件编译 + 链接**：复用 `Project/build.py` 的编译器与全部编译选项，
   `bsp_can.c / bcm_door.c / bcm_task.c` 及改动的既有文件另加 `-Wextra -Wshadow`；
2. **回环分支强制编译**：以 `-DCAN_LOOPBACK_SELFTEST=1` 再编一遍 `bsp_can.c / bcm_task.c`
   ——该分支在默认配置下是**从不参与构建的死代码**，不拉进来就永远没被验证过；
3. **协议层 PC 单测**：见 §8.4。

> ⚠️ **基线限制**：仓库按第三方许可合规要求排除了 Motion_driver eMPL 源码
> （`inv_mpu.h` 等），因此 `User/Modules/mpu6050/MPU6050.c` 在本机**无法编译**
> ——这是改动前的既有状态，`Project/build.py` 直接跑会在此处中断。
> 复现本次验证：`Project/stub_verify/build_verify.py` 用 5 个 MPU6050 桩符号
> 替换该文件，其余 100% 为工程真实源码：
>
> ```bash
> cd Project/stub_verify && python build_verify.py
> ```
>
> 产物落在 `Project/stub_verify/build/`，不污染正式构建（`Build/`）。

### 8.2 两级验收

**阶段一：回环自检（无对端节点、无分析仪也能验）**

1. `User/board_config.h` 置 `CAN_LOOPBACK_SELFTEST  1`；
2. 触发一条指令：让控制任务进入 `SAFE_BRAKE/SAFE_ERROR`（见 §6.1），
   或在任意任务里直接调 `BCM_RequestDoorLock(DOOR_MASK_ALL, DOOR_ACT_LOCK)`；
3. USART2 应出现两行：`BCM TX SEQ<n> D<mask>`，
   随后紧接 `BCM SELF-RX SEQ<n> D<20>`（回环分支在收到自发帧时打印）；
4. 这条 `SELF-RX` 才是证据：它证明
   **组帧 → 发送邮箱 → CAN 控制器 → 接收 FIFO → ISR → 队列 → 任务**整条链路通畅
   （只有 `TX` 只能说明组帧成功、邮箱写进去了，说明不了后半段）；
5. 验证后**务必改回 0**，否则永远收不到真实从站报文。

**阶段二：实机/分析仪**

- USB-CAN 分析仪（PCAN/周立功等，500 kbit/s，采样点 75% 左右）并联总线；
- 调用 `BCM_RequestDoorLock(DOOR_MASK_ALL, DOOR_ACT_LOCK)`；
- 抓包对照 §3.3 的 ID / Data / DLC 三张表：**逐字节核对**；
- 人为断开 CAN_H 或拔掉 120 Ω 终端电阻 → 应看到
  `BCM RTX`（重发）→ `FAIL`，并 `Error_CountCode(ERR_CAN_TX_TIMEOUT) > 0`，
  即 ACK 缺失路径确实生效（反证：总线正常时不该出现 RTX）。

### 8.3 实测记录（本机，GCC 工具链）

| 项 | 结果 |
|---|---|
| 编译 + 链接 | ✅ 通过，`bsp_can.c / bcm_door.c / bcm_task.c` 加 `-Wextra -Wshadow` 后 **0 告警** |
| 新符号已入映像 | `BSP_CAN_Init/Send/Receive/RxFifo0ISR`、`BCM_Door_PackReq`、`BCM_Task` 均可见于 ELF 符号表 |
| 新增 Flash | ≈ **2.0 KB**（bsp_can 912 B + bcm_door 168 B + bcm_task 953 B，含引用的 StdPeriph CAN 部分） |
| 新增静态 RAM | 37 B（.bss） |
| 新增动态 RAM | ≈ **0.3 KB**（FreeRTOS heap：接收队列 8×20 B + 请求队列 4×4 B + 队列控制块） |
| 协议层单测 | **19/19 PASS**（见 §8.4：PC 上真实运行） |

> 验证方式：复用 `Project/build.py` 的全部编译选项，8 个文件额外加 `-Wextra -Wshadow`。
> 因 MPU6050 用桩替换，**上表的 Flash/RAM 绝对值不等于真实固件**，
> 但新增模块的相对开销、新符号链接状态与协议层断言结果都是真实的。

### 8.4 协议层主机单元测试（把文档和代码锁在一起）

`bcm_door.h` 只依赖整型宽度别名，不依赖任何 STM32 寄存器定义，
所以用一个 12 行的 `stm32f10x.h` 替身头文件就能把它搬到 PC 上**真跑**：

```
Project/stub_verify/test_protocol/
├─ stm32f10x.h          # 仅 typedef uint8_t/uint32_t... 的替身头
├─ test_bcm_door.c      # 19 条断言
└─ (PC gcc 编译并运行，无需复位/Download)
```

覆盖内容：**① 实例帧回归**——§3.3 写死的 `ID 0x0601 2007 / Data 0F 01 46 28`、
回执 `0x0430 2107`，连同 PRI/DA/SA/CMD/SEQ 五个位域的回拆值；
**② 非法参数必须被拒**——mask=0、mask 越界、duty>100、timeout=0、空指针；
**③ 脏数据防御**——CMD 不是回执、发给别的节点（串帧）、DLC 不符、空指针。

> 价值：以后任何人改了位域宽度、默认占空比或文档里的示例帧，
> 这里会立刻失败，而不是等到总线抓包才发现对不上。
> 命令同 §8.1：`cd Project/stub_verify && python build_verify.py`（单测在最后自动执行）。

---

## 9. 易错清单

| 坑 | 表现 | 处理 |
|---|---|---|
| **忘开 APB1 时钟** | `CAN_Init()` 直接返回 Failed | `RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE)` 必须在 Init 之前 |
| **只有一块板也要测** | 无 ACK → 发不出去 | Normal 模式下无应答节点会反复重发；用 `CAN_LOOPBACK_SELFTEST=1` 自检 |
| **终端电阻缺失** | 波形振铃、ACK Error、Bus-Off | 总线两端各 120 Ω |
| **ISR 优先级 < 11** | 中断里调 FromISR API 触发断言/异常 | 抢占优先级 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` |
| **回执窗口 < 执行时限** | 每条指令必然超时重发 | 窗口由 `BCM_AckWindow()` 现算：`timeout_10ms×10 + 200` |
| FIFO 溢出 | 丢帧且无感知 | 驱动内清 `FOV0` 并累计 `rx_drop`，可由 `BSP_CAN_GetCounters()` 读出 |
| 32 位毫秒回绕 | 长时间运行后超时判断失效 | 用有符号差值 `(int32_t)(now - deadline) >= 0` |
| PA11/PA12 接了 USB 电路 | 电平被拉、无法通信 | USB 未使用，硬件上断开相关上拉/连接 |
| 过滤器 ID 未左移 3 位 | 收不到任何帧 | 32 位掩码模式下 `(id<<3) | IDE | RTR` 才是寄存器格式 |
| Bus-Off 按**电平**记账 / 每拍重启 CAN | 20ms 任务下每秒 50 次 `Error_Record`，16 条环形历史被灌满，其它错误频次彻底失真 | 边沿触发：只在"进入"那一拍记账；强制重启节流到 1s 一次（`BCM_PollBusOff`） |
| 编译开关里的代码从未被编译 | `#if CAN_LOOPBACK_SELFTEST` 分支平时不参与构建，改坏了也不知道 | 宏用 `#ifndef` 包裹，验证脚本以 `-D` 强制编译该分支 |
