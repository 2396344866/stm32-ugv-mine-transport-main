---
status: active
owner: user
scope: 项目背景与设备定位、命名理由（背景侧）；技术架构见根 Readme.md
---
# 地下矿井轨道运输与状态监测终端

> 固件代号：`STM32F103RCT6_EdgeNode` ｜ 版本：`1.0.0` ｜ 主核：STM32F103RCT6（Cortex-M3，72MHz，256KB Flash / 48KB RAM）
>
> 本文档为应用固件说明，覆盖系统背景、井下工况定位、软件架构，以及 `User/app/` 应用层**每个文件夹、每个文件**的职责与关键接口。

---

## 一、项目命名与理由

### 推荐名称

**地下矿井轨道运输与状态监测终端**
（英文：*Underground Mine Rail-Transport & Condition-Monitoring Terminal*）

### 命名逻辑（结合地下矿井环境与轨道运输设备）

| 命名片段 | 依据 |
|---|---|
| **地下矿井** | 锚定部署环境——煤矿 / 金属矿地下开采巷道与运输系统，区别于通用监测设备 |
| **轨道运输** | 点明被控主体——井下单轨吊 / 卡轨车等轨道运输设备 |
| **状态监测** | 对应终端真实能力：压力 / 姿态多参数采集、边缘特征提取、迟滞诊断、数据回传地面监控 |
| **终端** | 设备形态——嵌入式现场节点，区别于上位机与云平台 |

### 备选名称

- **井下运输安全监控终端**（偏"控制 + 安全"）
- **矿井轨道运输测控节点**（偏监测，表述简洁）
- **基于 STM32 的井下设备状态监测装置**（突出状态监测主线）

---

## 二、背景与设备定位

### 2.1 地下矿井作业环境与需求

地下矿井（煤矿 / 金属矿）生产依赖井下巷道中的轨道运输系统完成物料、设备与人员的转运。运输车在起伏巷道中运行，面临以下典型风险：

- **坡道倾角**导致车体姿态变化与下滑风险，需实时测倾并主动调速；
- **运行速度失控**、载货摆动引发的脱轨 / 倾覆隐患；
- **关键管路压力异常**（液压支架工作阻力、排水管压力）预示顶板 / 设备故障；
- **井下空间封闭**，人员 / 车辆定位困难，事故救援依赖实时位置信息。

本终端以嵌入式节点形态安装在运输车或巷道控制点，持续采集速度、倾角、压力等参数，在端侧完成控制与初步诊断，并通过 Wi-Fi 上送至井下网关 / 监控平台，支撑运输安全与预测性维护。

### 2.2 终端功能映射（代码实体 → 矿井物理量）

| 代码能力 | 矿井对应物理量 |
|---|---|
| MPU6050 加速度 / 倾角（DMP） | 运输车 **倾角 / 姿态**（坡道平衡、防倾覆） |
| TIM2 正交编码器 | 运输车 **转速 / 车速** 测量 |
| TIM1 PWM + 方向 GPIO | 运输车 **调速 / 方向执行** |
| ADC1_CH4 压力变送器 | 井下 **液压 / 张力压力**（液压支架工作阻力 / 排水管压力代理，折算缆绳张力）——监测 |
| 安全状态机 | 超速 / 超倾角 / 张力异常 → **分级保护 + 电磁刹车** |
| UWB（BU03） | 井下 **人员 / 车辆 / 检修资产定位** |
| ESP8266 | 监测数据 **可选备份上行**（经矿井通信骨干回传地面监控站） |
| OLED + Key | 就地 **状态显示** 与 **人工复位** |
| RTC + STOP | 电池供电场景 **低功耗长期监测** |

---

## 三、系统总体架构

软件采用分层 + FreeRTOS 抢占式任务调度，按实时性分级：

```
┌───────────────────────────────────────────────────────────┐
│  应用任务层（FreeRTOS，优先级：控制 > 守护 > 监测 > 定位/通信 > HMI）│
│  Control_Task  Housekeep_Task  Monitor_Task  UWB_Task       │
│  Comm_Task     HMI_Task                                        │
├───────────────────────────────────────────────────────────┤
│  算法中间件层（app/algo）  PID / 滤波 / 迟滞报警 / 安全状态机 / 三边定位 │
├───────────────────────────────────────────────────────────┤
│  系统服务层（app/sys）    错误记录 / 健康管理 / 控制节拍调度            │
├───────────────────────────────────────────────────────────┤
│  板级驱动层（app/bsp）    ADC/EEPROM/ESP/电机/RTC/USART/UWB/看门狗/…  │
├───────────────────────────────────────────────────────────┤
│  第三方中间件（User/Modules）  MPU6050(eMPL/DMP) / DW1000          │
├───────────────────────────────────────────────────────────┤
│  硬件外设层（Core/FWLib）  TIM/ADC/I2C/USART/SPI/RTC/IWDG/EXTI/GPIO  │
└───────────────────────────────────────────────────────────┘
```

**任务划分与优先级**（见 `app/tasks/tasks.h`）：

| 任务 | 优先级 | 周期/触发 | 职责 |
|---|---|---|---|
| Control_Task | 5（最高） | 20ms 节拍信号量 | 双闭环 PID + 安全状态机（运输车调速 / 防倾覆 / 紧急刹车） |
| Housekeep_Task | 4 | 200ms | 看门狗喂狗 + 存活巡检 + 复位溯源 |
| Monitor_Task | 3 | 50ms | 压力采样→滤波→报警→落盘→上送 |
| UWB_Task | 2 | 200ms | TWR 测距 + 三边定位 |
| Comm_Task | 2 | 1000ms | 状态快照 + 日志经串口上报（ESP8266 可选备份上行） |
| HMI_Task | 1 | 100ms | OLED 多页显示 + 按键 |

---

## 四、应用层（`User/app/`）文件清单与职责

应用层按职责分为四类目录：**`algo/`（算法）、`bsp/`（驱动）、`sys/`（系统服务）、`tasks/`（任务）**。以下逐一列出每个文件。

### 4.1 `app/algo/` —— 算法中间件层（与硬件解耦的纯算法）

| 文件 | 类型 | 职责 | 关键接口 / 要点 |
|---|---|---|---|
| `pid.h` / `pid.c` | 算法库 | 串级双闭环 PID。提供**位置式**（`PID_Calc_Position`，用于外环姿态）与**增量式**（`PID_Calc_Incremental`，用于内环速度，返回增量由调用方累加）；含积分限幅（`integ_limit` 抗饱和）、输出限幅、`mode` 切换。 | `PID_Init` / `PID_SetTarget` / `PID_SetGains` / `PID_Reset`；结构体 `PID_Handle` 集中管理增益与限幅 |
| `filter.h` / `filter.c` | 算法库 | 混合数字滤波，**两级串联**：① **去极值平均**（8 槽缓冲去掉一个最大、一个最小后除 n−2，抗单点飞点/脉冲尖峰）；② **一阶滞后**（`y=prev+α(x−prev)`，抗工频与缓慢抖动）。顺序不可颠倒——先去极值再滞后，避免滞后把尖峰缓慢扩散成持续偏差。 | `Filter_TrimmedMean`、`Filter_FirstOrderLag`、`Filter_Update`（串联两者）；`Filter_Handle` 含 8 槽环形缓冲与 `alpha` 系数。压力通道 `alpha=0.2`（50 ms 采样，τ≈12.5 ms）；倾角通道 `alpha=0.3`（20 ms 节拍，τ≈60 ms），见 `control_task.c` 的 `ANGLE_FILTER_ALPHA` |
| `alarm.h` / `alarm.c` | 算法库 | 迟滞报警状态机。区分 `ALARM_NORMAL / WARN / CRIT`；以"进入阈值宽、退出阈值窄"的迟滞带避免临界点抖动误报。 | `Alarm_Init(ref, warn_enter, warn_exit, crit_enter)`、`Alarm_Update`、`Alarm_DeviationRatio` |
| `safety_fsm.h` / `safety_fsm.c` | 算法库 | 多级安全状态机：`SAFE_RUN / WARN / BRAKE / ERROR`。三重抗误动：① **N-of-M 越限确认**（默认 5 拍中 3 拍越限才升级，单点尖峰不动作）；② **迟滞回差**（enter/exit 阈值分离，倾角 25° 进 / 22.5° 出）；③ **失效保持**（传感器失效按 fail-safe 视为越限，不当作 0 放行）。`BRAKE` 与 `ERROR` 为**锁定态**，需显式清除。 | `Safety_Init`、`Safety_Update(speed, angle, tension, SafetyInval*)`、`Safety_ClearError`、`Safety_Tripped` |
| `uwb_2d.h` / `uwb_2d.c` | 算法库 | 二维定位解算。`UWB_Trilaterate2D` 以三基站坐标与三测量距离做最小二乘三边定位，解算标签坐标（米）。 | 输入 `anchors[3]`、`d[3]`；输出 `Point2D`，返回 0 成功 |

### 4.2 `app/bsp/` —— 板级支持 / 外设驱动层（直接操作 STM32 外设）

| 文件 | 类型 | 职责 | 关键接口 / 要点 |
|---|---|---|---|
| `bsp_adc.h` / `bsp_adc.c` | 驱动 | 压力变送器 ADC 采集。配置 `ADC1_CH4`（PA4，硬件 RC 低通 + TVS 前端），量程 0~10 bar，标称 5 bar。 | `BSP_ADC_Init`、`BSP_ADC_ReadRaw`（软件触发单次 12 位 0..4095） |
| `bsp_at24c256.h` / `bsp_at24c256.c` | 驱动 | AT24C256 EEPROM（I2C2，与 MPU6050 同总线，地址 `0xAE`）。用于监测记录本地断点续传。写后按页分块并轮询器件 ACK 等 tWR，读写均返回真实错误码。 | `BSP_AT24C256_Init`、`BSP_AT24C256_Write/Read` |
| `bsp_esp8266.h` / `bsp_esp8266.c` | 驱动 | ESP8266 Wi-Fi **可选**上行（USART1，AT 指令）；未配置密钥时自动降级为仅调试串口。 | `BSP_ESP8266_Init`、`ESP8266_ConnectWiFi`、`ESP8266_OpenTCP`、`ESP8266_Send`；凭证经编译期宏占位，不内嵌真实密钥 |
| `bsp_motor.h` / `bsp_motor.c` | 驱动 | 电机驱动：TIM1_CH1 PWM 调速、PB12/PB13 方向、TIM2 正交编码器读增量、状态指示（黄灯/蜂鸣/红灯）。 | `BSP_Motor_Init`、`BSP_Motor_SetDuty(0..999)`、`BSP_Motor_SetDir`、`BSP_Encoder_Reset`、`BSP_Encoder_ReadDelta`、`BSP_Status_Set` |
| `bsp_rtc.h` / `bsp_rtc.c` | 驱动 | RTC（LSI）低功耗时基。支持 STOP 模式唤醒，用于电池供电长期监测。 | `BSP_RTC_Init`、`BSP_RTC_GetTime`、`BSP_RTC_SetWakeupSeconds`、`BSP_EnterStopUntilWakeup` |
| `bsp_usart.h` / `bsp_usart.c` | 驱动 | 通用串口驱动。多 USART 初始化、环形缓冲非阻塞收发、带超时读取；各 USART 中断统一经 `BSP_USART_IRQHandler` 处理（底半部原则）。 | `BSP_USART_Init(usart, baud, remap)`、`BSP_USART_Send/SendString`、`BSP_USART_ReadAvail`、`BSP_USART_Read(timeout)` |
| `bsp_uwb.h` / `bsp_uwb.c` | 驱动 | 集成安信可 BU03 UWB 模组（基于 DW3000，UART-AT，USART3 部分重映射 PC10/PC11）。TWR 测距在模块内完成，主机解析距离帧。 | `BSP_UWB_Init`、`UWB_StartRanging`、`UWB_ReadDistance(dist_m)` |
| `bsp_watchdog.h` / `bsp_watchdog.c` | 驱动 | 独立看门狗 IWDG。超时未喂狗即系统复位。 | `BSP_IWDG_Init(timeout_ms)`、`BSP_IWDG_Feed` |
| `Delay.h` / `Delay.c` | 驱动 | 基于 DWT 的精确延时，作为其他驱动时钟基准（须先于任何 `Delay_ms` 调用 `Delay_Init`）。 | `Delay_Init`、`Delay_ms`、`Delay_us` |
| `Key.h` / `Key.c` | 驱动 | 独立按键扫描，供 HMI 翻页与清除安全锁定。 | `KEY_GPIO_Config`、`Key_GetNum`（1=翻页，2=清除锁定） |
| `LED.h` / `LED.c` | 驱动 | 状态 LED 驱动（绿=心跳，蓝=控制环存活，红=故障锁定）。 | `LED_GPIO_Config`、`LED_Green`、`LED_Blue` 等 |
| `oled.h` / `oled.c` | 驱动 | 0.96" OLED（SSD1306，模拟 I2C）显示驱动，多页状态呈现。 | `OLED_Init`、`OLED_Clear`、`OLED_ShowString`、`OLED_ShowNum`、`OLED_ShowFloat`、`OLED_ShowSignedFloat/Num` |
| `OLED_Font.h` | 数据 | OLED 字模库（8×16），供 `oled.c` 取模；用 `__GNUC__` 包裹 GCC 专属 pragma，跨编译器安全。 | — |

### 4.3 `app/sys/` —— 系统服务层（跨任务的公共机制）

| 文件 | 类型 | 职责 | 关键接口 / 要点 |
|---|---|---|---|
| `error.h` / `error.c` | 服务 | 统一错误记录。`SysErrCode` 枚举覆盖 ADC/I2C/EEPROM/UWB/ESP/MPU/RTC/看门狗复位等；供健康管理任务消费。 | `Error_Record`、`Error_CountTotal`、`Error_CountCode`、`Error_Last`、`Error_Clear` |
| `health.h` / `health.c` | 服务 | 任务健康管理。各任务周期性 `Health_Report`；`Health_Check` 检测是否在窗口（1s）内汇报存活（卡死检测）。 | `Health_Init`、`Health_Register`、`Health_Report`、`Health_Check`、`Health_IsAlive`、`Health_UptimeSec` |
| `scheduler.h` / `scheduler.c` | 服务 | 控制节拍调度。配置 TIM3 20ms 定时中断并创建 `xControlSemaphore`；中断中释放信号量驱动控制任务确定性节拍（底半部原则）。 | `Scheduler_Init`、`Scheduler_ControlTickISR`（在 TIM3 更新中断中调用） |

### 4.4 `app/tasks/` —— FreeRTOS 应用任务层（抢占式，按实时性分级）

| 文件 | 类型 | 职责 | 关键要点 |
|---|---|---|---|
| `tasks.h` | 公共头 | 任务层中枢：任务 ID / 优先级 / 栈深定义；全局 `SystemState`（跨任务共享，互斥访问）、`xSystemStateMutex`、`xLogQueue`；各任务入口声明；`Control_ClearSafetyError` 供 HMI 清除锁定；`Monitor_LogRead` / `Monitor_LogHead` 供通信任务做断点续传。 | `SystemState` 含控制/监测/定位/系统级字段；访问须持 `xSystemStateMutex` |
| `control_task.c` | 任务 | **实时控制（最高优先级）**。等待 20ms 节拍信号量 → 采样编码器速度 + MPU6050 倾角（**先经一阶滞后 α=0.3 平滑再进安全状态机**，τ≈60ms）→ 先跑安全状态机 → 再跑双闭环级联 PID（外环姿态 PD + 内环速度 PI 增量式，含积分限幅抗饱和）→ 输出 PWM/方向。BRAKE/ERROR 锁定输出 0。 | 浮点运算全在任务上下文（中断仅释放信号量）；状态指示：WARN→黄灯，ERROR/BRAKE→红灯；张力由监测子系统压力折算作为安全代理量；`SafetyInval` 上报 DMP 失效供 fail-safe 判定 |
| `monitor_task.c` | 任务 | **监测**。ADC 采样（4~20mA 电流环变送器，标定 raw≈496~2481 对应 0~10 bar）→ 区间外失效判据 → 两级混合滤波 → 迟滞报警 → 张力代理 → **RAM 黑匣子环形缓冲**（每拍预存，60 条≈3 s）→ **三层自适应落盘**（30s/10s/50ms）→ 推送 `xLogQueue`；支持 RTC 时标与 STOP 低功耗（按需开启）。详见 §4.5。 | 采样 50 ms；常态 30s 一条、趋势可疑 10s、异常 50ms 全速黑匣子；head 寿命 694/231 天 |
| `comm_task.c` | 任务 | **通信**。周期生成系统状态快照经 `DBG_USART` 调试串口上报；ESP8266 Wi-Fi 为可选备份上行（状态 + 日志队列），未配置密钥时降级仅串口。 | 状态行含安全态/报警/气压/速度/坐标/错误数；凭证编译期占位符 |
| `uwb_task.c` | 任务 | **定位**。周期性对三固定基站 TWR 测距 → `UWB_Trilaterate2D` 解算二维坐标 → 回写全局状态；测距失败记错（重试 3 次）。 | 基站坐标按现场标定；`USE_UWB_SUBSYS` 关闭时退化为仅心跳 |
| `hmi_task.c` | 任务 | **人机交互**。OLED 四页显示（总览 / 控制 / 监测 / 定位）；KEY1 翻页、KEY2 清除安全锁定；LED 心跳/运行指示。 | 显示经由 `xSystemStateMutex` 拷贝快照，避免临界区过长 |
| `housekeep_task.c` | 任务 | **系统守护**。IWDG 喂狗（周期 200ms ≪ 超时 1s）+ `Health_Check` 任务存活巡检 + 复位溯源（IWDG 复位标志记 `ERR_WATCHDOG_RESET`）；刷新全局 uptime/错误计数。 | 任一高优先级任务长期卡死 → 本任务无法及时喂狗 → 硬件复位自愈 |

### 4.5 EEPROM 环形日志布局与掉电安全（`monitor_task.c`）

**地址布局**（AT24C256，页长 64 B，总容量 32 KB）

| 区域 | 地址 | 物理页 | 容量 | 内容 |
|---|---|---|---|---|
| head A | `0x0000` | **第 0 页** | 5 B | `seq(4, LE)` + `crc8(1)` |
| head B | `0x0040` | **第 1 页** | 5 B | `seq(4, LE)` + `crc8(1)` |
| 日志区 | `0x0080`–`0x087F` | 第 2–33 页 | 128 × 16 B = 2048 B | 环形记录 |
| — | — | — | 合计 2058 B（占 32 KB 的 6.3%） | — |

> head A/B **跨 64 B 物理页**的原因见下方「head 跨 64B 物理页」一节——是**掉电物理隔离**，不是延寿。

**单条记录 16 B逐字段分配**（`monitor_task.c` `eeprom_log`）

| 偏移 | 长度 | 字段 | 取值 |
|---|---|---|---|
| `rec[0]` | 1 | seq | `head_seq & 0xFF`，128 槽循环 |
| `rec[1..3]` | 3 | 时标 hour/min/sec | 来自 RTC；心跳 30s 下覆盖时长 64 min，跨天概率极低，故不存日期 |
| `rec[4]` | 1 | alarm | `Alarm_Update` 输出：0=NORMAL / 1=WARN / 2=CRIT |
| `rec[5..6]` | 2 | raw | ADC 12 位原始值 0–4095，小端 |
| `rec[7..8]` | 2 | filt100 | **滤波后气压 ×100**（0–1000 对应 0–10 bar），分辨率≈0.01 bar |
| `rec[9..14]` | 6 | 预留扩展 | 当前填 0；预留毫秒 / 温度 / 故障码 |
| `rec[15]` | 1 | crc8 | `crc8_atm(rec[0..14])`，poly=0x07 init=0x00 |

> **有效数据 9 B + 预留 6 B + CRC 1 B = 16 B。** 其中滤波值以「浮点 ×100 的整数」落盘而非原始 float：既省 2 B，又让解析端无需处理与平台相关的浮点字节序。
> **预留 6 B 并非凑数**：记录长度必须整除页长 64 B 才能保证单条永不跨页——实测 10 B 与 12 B 均会跨页（12 B 时 128 槽中有 16 槽跨界），16 B 时每页整放 4 条、共 32 页。多出的 6 B 正好留作扩展字段，将来加字段无需重排布局或重算跨页。

**head 跨 64B 物理页：为了掉电物理隔离，不是为了延寿**

AT24C256C datasheet §7.4 Write Cycle Timing 明确写道：在 tWR（≈5 ms）内部自定时写周期内，*"any attempts to read from or write to the memory array will not be processed"*，正在被改写的存储单元处于**不确定态**。

若 head A 与 head B 同处一个 64B Page Latch，则写 A 期间掉电，**同一页参与该次高压写周期的数据会一起变不确定**——A/B 双份会同时报废，双份冗余形同虚设。故布局为：

| 区域 | 地址 | 物理页 | 内容 |
|---|---|---|---|
| head A | `0x0000` | **第 0 页** | `seq(4, LE)` + `crc8(1)` = 5 B |
| head B | `0x0040` | **第 1 页** | `seq(4, LE)` + `crc8(1)` = 5 B |
| 日志区 | `0x0080`–`0x087F` | 第 2–33 页 | 128 × 16 B = 2048 B |

三块区域分处**互不相邻的物理页**，任一次写周期掉电只波及一块，A/B 双份的冗余才真正成立。

> **必须与"寿命"分开讲**：Write Endurance = 1,000,000 Write Cycles 该指标是**按字节（cell）计**的，64B 页只是 I2C 写缓冲的传输粒度，擦写只作用于目标字节的浮栅。因此就**寿命**而言，同页的不同字节各享独立预算，跨页并不额外延寿。跨页换来的是**掉电物理隔离**，若按"延寿"来讲反而会把 EEPROM 机理讲错。

**降频：寿命维度的真正手段**

A/B 双份只解决冗余，不解决"写得太频繁"。head 寿命 = endurance × 写间隔：

| 落盘策略 | head A/B 写间隔 | head 寿命 | 日志区寿命 | 系统寿命 |
|---|---|---|---|---|
| 1 s 一条（改造前） | 2 s | **≈23 天** | ≈4.1 年 | ≈23 天 |
| **30 s 心跳（本工程）** | 60 s | **≈694 天** | ≈122 年 | **≈1.9 年** |

系统寿命被最短腿（head）决定，故降频收益是决定性的：**23 天 → 694 天，30 倍**。`Project/stub_verify/crc_check.py` 从源码宏实时核算这两行数字，若心跳被改回 1 s 会直接报 FAIL。

**两级落盘触发：RAM 黑匣子 + 事件刷盘**

单纯降频会牺牲故障现场分辨率，故用两级触发把「寿命」与「黑匣子完整性」解耦：

1. **正常态**：`MONITOR_PERIOD_MS = 50 ms` 采样一次，每拍都只写入 RAM 环形缓冲（`MONITOR_RING_N = 60` 条 × 16 B = 960 B，约 **3 s 现场数据**），**不动 EEPROM**；按上面的三层自适应节拍落盘。
2. **异常态**：`alarm != ALARM_NORMAL` 或 `safety != SAFE_RUN` 或 ADC 失效时置位 `s_blackbox`（锁存），每拍调用一次 `ring_flush_step()`。
3. **每次 step 只提交一次 head**：head 寿命按「调用次数」计而非「条数」。

**三层自适应落盘：用「数据价值」决定采样密度**

单一节拍无法同时满足「省寿命」与「够取证」：常态用 30 s 能省寿命，但**缓慢渐变故障**（管路泄漏、闸阀缓卡）在到达报警阈值前要几分钟，30 s 采样只剩几个孤立点，无法还原演化过程；反过来常态就用 50 ms 落盘，head 寿命又退回 23 天。

故按偏差自动分三档：

| 档 | 触发条件 | 落盘节拍 | head 寿命 | 覆盖时长 |
|---|---|---|---|---|
| ① 正常 | 偏差 ≤ 5% | **30 s** | ≈694 天 | 128×30s ≈ 64 min |
| ② 趋势可疑 | 偏差 > 5%，未报警 | **10 s** | ≈231 天 | ≈21 min |
| ③ 异常 | 报警越限 / 安全态非 `SAFE_RUN` / ADC 失效 | **50 ms 全速黑匣子** | 分步摊薄 | 3 s 全分辨率 |

> ②档取 10 s 而非 5 s 是权衡结果：5 s 加密时 head 寿命仅 **116 天**，不足以支撑长周期运行；10 s 是「能看清趋势」与「够长寿命」的折中。偏差判据用**滤波后**值，避免噪声把节拍一直拉高。
> 配合 RAM 环形缓冲（`MONITOR_RING_N = 60`，每拍预存 ≈3 s 现场），③档触发时能一次性无损刷出故障前后连续曲线；`s_blackbox` 锁存保证故障持续期间持续刷到缓冲清空。

**气压采样：4~20mA 标定与失效判据**

变送器按**4~20 mA 电流环**配置（井下牵引/制动管路的标准做法）：① 恒流驱动、信号强度与传输距离无关，抗巷道变频器干扰；② **可检测断线**（低于 3.6 mA 即故障），而电压式断线输出 0V 与真实 0 压力无法区分；③ 两线制本安回路共用一对线。

标定链路：采样电阻 200 Ω + 2:1 分压 ⇒ `V_ADC = 0.4~2.0 V` ⇒ **`raw ≈ 496`（0 bar）～`2481`（10 bar）**，故换算式必须带零点偏置：

```
bar = (raw - 496) / (2481 - 496) × 10.0
```

有效区间约 2233 counts ⇒ **分辨率 ≈ 5 mbar/LSB**（0.05%），满足要求。

**失效判据（原实现的真实漏洞，已修）**：旧代码写 `raw == 0` 只 `Error_Record` 不报警，且换算式 `raw/4095 × VREF/VREF × FS` 中 **VREF 自我相消**，等价于假设「变送器满量程恰好 3.3 V」。现改为区间外判据：

| 条件 | 含义 | 处理 |
|---|---|---|
| `raw < 446` | 电流环开路 / 断线 | 置失效、记错、**该拍不进滤波** |
| `raw > 2679` | 超量程（传感器故障/短路/超压） | 同上 |
| 区间内 | 正常 | 正常标定 |

> ⚠️ **失效拍不送入滤波**是关键：否则一个坏值会被两级滤波当成正常值吃掉，**故障被"抹平"而失去告警**。失效拍代表值取**量程下限**（气压过低=张力不足=危险），而非 0 bar 当作正常。
> 失效状态经 `g_state.pressure_bad` 上报，`control_task` 填入 `SafetyInval.tension_bad`，由安全状态机按 **fail-safe 处理为最保守值**——杜绝「读不到 = 0 = 安全」的放行路径。

断言见 `Project/stub_verify/test_pressure_scale.c`（21 条：标定端点/单调性/分辨率/失效判据/分层节拍）。

**制动优先级：不挤占 ≤20 ms 响应窗口**

EEPROM 单条写入含 tWR ≈ 5 ms 内部写周期。若一次刷满 60 条，**单次调用将连续阻塞约 300 ms**——是制动窗口的 15 倍，构成安全隐患。三重保障：

| 保障 | 做法 |
|---|---|
| ① 任务优先级 | `TASK_PRIO_CONTROL(5) > TASK_PRIO_MONITOR(3)`，且控制环由 TIM3 中断释放信号量驱动；FreeRTOS 抢占式调度下 control_task 必然先拿到 CPU，monitor_task 是**被抢占**而非**拖延控制环** |
| ② 分步执行 | `ring_flush_step()` 每次最多 `RING_FLUSH_BATCH = 2` 条，单次最坏阻塞 **2×5 = 10 ms < 20 ms**，不存在长阻塞窗口 |
| ③ 无忙等 | 每步写完即返回到 `vTaskDelay`，主动让出 CPU |

> ⚠️ **关键约束**：刷盘触发判据里**不能包含"制动正在动作"**。实现仅以 `alarm` 越限 / 安全态非 `SAFE_RUN` 为触发（故障发生后尽快取证），制动动作本身由 control_task 在自己的 20 ms 节拍内独立完成，**不依赖 monitor_task**。上述阻塞预算由 `test_log_ring.c` 断言守护。

**掉电安全：三重保护**

1. **写序保证 head 是提交点**：先写记录 → `seq++` → 提交到另一份 head。记录写失败或 head 提交失败均**回退 `seq`**，保证 head 始终与 EEPROM 中的有效状态一致。掉电落在记录写窗口 ⇒ head 未推进 ⇒ 该槽尚未提交，下次直接覆盖。
2. **head A/B 双份交替**：每提交一批翻转写入另一份，两份**分处第 0 / 第 1 物理页**（跨页理由见上方「head 跨 64B 物理页」）。掉电最多损坏非活动那份，活动份完好——若同页则两份可能一起变不确定。
3. **每条记录自带 seq + CRC-8**：可逐条判定完整性，而非仅按位置推定。

**上电恢复与续传**

- `head_load()`：读 A/B 两份，**crc 不符者直接判废**，取「校验通过且 seq 较大」者；新旧比较用模 2³² 无符号差值 `(b-a) < 0x80000000`，正确处理 seq 回绕。
- `Monitor_LogRead(back, out, len)`：自最新一条向旧扫描，逐条校验 `crc8` 与 `seq` 连续性，任一不符即 `continue` 跳过（不中断扫描），**撕裂条目被精确识别并丢弃，不会被当作有效历史**。
- `Monitor_LogHead()`：返回已提交 head，即续传起点。

---

## 五、支撑文件（位于 `User/`，非 app 层但为固件入口与配置）

| 文件 | 职责 |
|---|---|
| `main.c` | 固件入口：`System_Init` 硬件初始化；创建互斥量/队列；`Scheduler_Init`；启动 6 个任务；FreeRTOS 钩子（idle 省电 `__WFI`、栈溢出、malloc 失败）。 |
| `board_config.h` | 统一板级配置：FW 名/版本、特性开关（`USE_*`）、引脚资源表、控制节拍 20ms。所有驱动/任务从此取引脚定义，避免分散硬编码。 |
| `stm32f10x_it.c` / `.h` | 中断服务：转发 `SVC/PendSV/SysTick` 至 FreeRTOS；各 USART 中断调 `BSP_USART_IRQHandler`；TIM3 中断调 `Scheduler_ControlTickISR`。 |
| `FreeRTOSConfig.h` | FreeRTOS 内核配置（任务数、优先级、堆等）。 |
| `syscalls.c` | 标准库 retarget：ARMCC 关半主机 + `_sys_write` 重定向 `printf` 至 `DBG_USART`；GCC 分支保留 newlib `_sbrk`，双编译器兼容。 |
| `dwt.h` | DWT 周期计数器宏，供 `Delay_Init` 基准。 |

### `User/Modules/`（第三方/厂商中间件，被 app 引用）

| 目录 | 内容 |
|---|---|
| `mpu6050/` | MPU6050 及官方 eMPL/DMP 运动驱动：`inv_mpu.c`、`inv_mpu_dmp_motion_driver.c`、`mpu_port.c`、`MPU6050.c/.h`、寄存器与 DMP 密钥表。 |
| `dw1000/` | DW3000 寄存器与 API（目录名沿用 dw1000，实为 DW3000 驱动，休眠未激活）：`deca_device.c/.h`、`deca_param_types.h`、`deca_regs.h`、`deca_types.h` 等。 |

---

## 六、关键设计要点

### 6.1 实时性
- **20ms 确定性控制节拍**：TIM3 硬件定时中断释放二值信号量，控制任务据此同步，控制周期抖动最小。
- **中断底半部**：编码器捕获、USART 接收等中断仅做"置标志 / 入队"轻量处理，重活交任务上下文，缩小临界区。
- **时间触发 + 事件驱动混合**：控制回路时间触发保证确定性；监测/定位/HMI 事件/定时驱动兼顾效率。

### 6.2 高稳定性
- **多级安全状态机**：超速 / 超倾角 / 张力异常分级进入 WARN→BRAKE→ERROR，BRAKE/ERROR 锁定需显式清除。
- **看门狗 + 存活巡检**：IWDG 独立看门狗由守护任务喂狗；`Health_Check` 检测任务卡死，超时即硬件复位自愈。
- **统一错误记录与复位溯源**：`Error_Record` 分级记录，`ERR_WATCHDOG_RESET` 标记复位来源，便于现场排查。
- **数据断点续传**：监测记录落盘 EEPROM，断网/复位后可续传，避免数据丢失。
- **鲁棒报警**：迟滞报警（进入 ±5% / 退出 ±2%）避免临界抖动误报；PID 积分限幅 + 输出限幅抑制超调与饱和。
- **低功耗长期监测**：RTC 闹钟唤醒 + STOP 模式 + MOS 高侧动态关断外设，适配电池供电部署。

### 6.3 UWB 定位为什么不接安全状态机（防误动设计）

井下巷道狭长、多径效应严重，BU03（DW3000）走 UART-AT 指令测距，**测距在模块内完成、主机只解析结果帧**，链路latency 与串口阻塞不确定；测距跳变或基站丢包是常态而非异常。工程上的判断是：**UWB 不参与安全链**。

**代码依据**（`control_task.c`）：安全状态机只吃三个量

```c
SafetyInval inv = { 0u, (uint8_t)!s_mpu_ok, pressure_bad };
SafetyState st  = Safety_Update(&s_safety, speed, angle, tension, &inv);
```

即 `Safety_Update(speed, angle, tension)`。`uwb_task.c` 只写 `g_state.position / uwb_d[0..2] / uwb_valid`，**`uwb_valid` 唯一的消费者是 HMI 的 OLED 显示**，`control_task` 从不读它。因此 UWB 跳变或丢包在架构上**不可能**触发 BRAKE——不是靠滤波"防住"的，是靠**数据流隔离**从原理上排除的。

> 面试时必须点明这一层：**UART-AT 的延迟不可控 ⇒ 让它进入安全链就是引入了不可控的输入通路 ⇒ 架构上把它摘出去，比在滤波器上花力气更根本。**

代价与补偿：定位精度不可用于安全，故安全判据全部改用**车上可直接测量的物理量**——编码器测速（`TIM2` 正交解码）、MPU6050 倾角、气压折算张力。UWB 只承担"人在哪"的运维/调度价值。

**真正会触发 BRAKE 的三条通路及其防呆**（`safety_fsm.c`）：

| 通路 | 误动风险 | 三重抗误动 |
|---|---|---|
| 倾角（MPU6050 / DMP） | DMP 单点跳变、井下振动干扰最大 | ① **一阶滞后 α=0.3**（20ms 节拍下时间常数 ≈60ms）压尖峰；② **N-of-M 5 取 3**（`SAFETY_NOM_N=3/M=5`）；③ **迟滞 25° 进 / 22.5° 出** |
| 速度（编码器） | 打滑、脉冲干扰 | 同 N-of-M + 迟滞 10%（`120 进 / 108 出`） |
| 张力（气压折算） | 变送器断线、超量程 | **区间外失效判据** → `g_state.pressure_bad` → `SafetyInval.tension_bad` → fail-safe |

三层语义（`safety_fsm.h` / `.c` 注释）：

1. **N-of-M 越限确认**：M 拍滑动窗口内至少 N 拍越限才升级。单点尖峰不再直接锁死。窗口用**滑动**而非"连续 N 拍"，否则长时间异常工况下计数会被偶发正常拍打断而永不动作。
2. **迟滞回差**：`exit = enter × (1 - hyst)`，进入/退出阈值分离，消除阈值附近抖动翻转。
3. **失效保持（fail-safe）**：传感器失效时喂 `SENTINEL = 1.0e9f` 哨兵值喂入比较器，必然越过 enter 阈值，再经 N-of-M 确认后升锁——**杜绝「读不到 = 0 = 安全」的放行路径**。

另有两道锁存保护：

- `BRAKE/ERROR` 一旦进入即**锁定**，需 HMI 显式 `Safety_ClearError` 才复位，且复位时 `nom_init` 清空全部越限确认器，避免残留计数导致二次触发；
- 进入锁存的**边沿**才向 CAN 车身请求一次全车落锁（`s_prev_safe != st`），稳态不重复发帧。

**关于「20 ms 电磁刹车」的措辞纠正**：本工程 20 ms 是 `CONTROL_PERIOD_MS` **控制节拍**（`board_config.h`），BRAKE 的执行动作是把 `duty_out = 0`（`control_task.c:173`）并复位 PID，即**切断电机驱动**；CAN 落锁走 bcm 任务（20 ms 独立节拍）。本仓 `bsp_motor` 无独立电磁刹车通电时序定义，故**不可说「20 ms 电磁刹车动作」**——正确表述是「20 ms 控制节拍内完成安全判定并将驱动占空比置 0，锁定态需人工清除」。

---

## 七、构建与运行

- **IDE**：Keil MDK（ARMCC V5），工程文件 `Project/stm32f103RCT6.uvprojx`；FreeRTOS 使用 RVDS/ARM_CM3 移植层。
- **验证构建（GNU）**：`Project/build.py` 直接驱动 `arm-none-eabi-gcc`，产物 `Build/firmware.elf/.hex/.bin`。
- **运行前提**：`MicroLIB` 保持关闭（调试串口 `printf` 含 `%f` 浮点，MicroLIB 不支持）；`syscalls.c` 已实现 ARMCC 完整库 retarget。
- **调试串口**：`DBG_USART`（USART2，PA2/PA3，115200bps）输出状态快照与日志，便于现场诊断。

---

## 八、扩展指引

- **新增传感器**：在 `bsp/` 增加驱动，于 `board_config.h` 配置引脚，在 `Monitor_Task` / `Control_Task` 中接入 `SystemState`。
- **调整控制对象**：修改 `control_task.c` 顶部增益与安全阈值宏；算法本体（`pid.c`/`safety_fsm.c`）无需改动。
- **裁剪子系统**：通过 `board_config.h` 的 `USE_*` 开关关闭 UWB / ESP / HMI 等，减少资源占用。
- **现场标定**：UWB 基站坐标（`uwb_task.c` 中 `s_anchors`）、压力量程与标称值（`monitor_task.c`）按实际工况标定。

---

## 九、简历要点核对（证据化数据支撑）

将简历拟用表述与工程代码逐条对照，标注验证状态：**✅ 代码可证 / 用户实机验证（Keil 烧录）**。

> **简历拟用表述（外部引用，非本工程自述）**
> - 面向井下轨道运输的智能控制系统｜项目负责人｜https://github.com/2396344866/stm32-ugv-mine-transport｜2024.10-2025.03
> - 基于 STM32F103RCT6 + FreeRTOS 10.5.1 的嵌入式控制系统，集成运动控制（串级 PID）/ 状态监测（压力·张力监测）/ 无线定位（安信可 BU03（基于 DW3000）UWB 三点定位，车上本地解算）三大功能为统一固件，GCC 与 Keil 双工具链编译并实板烧录验证
> - Simulink 验证串级 PID 零超调、坡变 1.5 s 恢复，实现坡道平稳调速；四态安全状态机 + 独立看门狗 + 任务存活巡检保障端侧可靠；UWB 定位数据经矿井无线通信骨干回传地面调度室 QT 上位机监控

| 简历表述 | 状态 | 代码证据 |
|---|---|---|
| 6 个 FreeRTOS 任务（控制/监测/定位/通信/HMI/看家） | ✅ | `tasks.h`：TASK_ID_CONTROL/MONITOR/UWB/HMI/COMM/HOUSEKEEP 共 6；优先级 5/3/2/2/1/4 |
| 三子系统统一 FreeRTOS 架构 | ✅ | 控制 `control_task`、监测 `monitor_task`、定位 `uwb_task` 同工程协同调度 |
| GCC/Keil 双工具链编译链接 | ✅ | `Project/build.py`（arm-none-eabi-gcc，产物 .elf/.hex/.bin）；Keil `stm32f103RCT6.uvprojx` + RVDS 移植层 + `syscalls.c` ARMCC retarget |
| GCC/Keil 双工具链 + STM32 硬件烧录与台架验证 | ✅ | Keil ARMCC V5 工程已编译并**烧录至真实 STM32F103RCT6 板卡**（用户实机烧录运行）；并搭建台架完成坡道带载运行验证，实机调速表现与 Simulink 建模预期一致；GCC 工具链经本机 `build.py` 编译验证（零警告）。双工具链编译链接 + 硬件烧录 + 台架运行验证均已完成 |
| 统一 MPU6050 DMP 姿态解算 | ✅ | `Modules/mpu6050` + `inv_mpu.c`（`#define MPU6050`）经 I2C2(PB10/PB11) 采集，DMP 输出欧拉角/四元数；工程姿态解算统一为 MPU6050 单 IMU 方案 |
| Simulink 串级双闭环 PID（外 PD + 内 PI） | ✅ | `PID/Readme_PID.md` 完整建模（Simulink 模型用外环 Kp=2 Kd=0.5）；**实机部署最终整定**为外环 Kp=5 Kd=2、内环 Kp=12 Ki=0.5（`control_task.c` 宏定义） |
| 平直段零稳态误差 / 零超调 | ✅ | PID 文档仿真结论 |
| 坡度 0.26rad→1.5s 恢复；风载→2.0s 收敛 | ✅ | PID 文档：坡度突变 1.5s 内恢复（跌落<15%）；风流扰动摆角 2.0s 收敛至<0.02rad |
| 多级安全状态机 RUN/WARN/BRAKE/ERROR | ✅ | `safety_fsm.h`：SAFE_RUN/WARN/BRAKE/ERROR 四态，BRAKE/ERROR 锁定需显式清除；含 N-of-M 确认（3/5）、迟滞回差、失效保持 |
| 独立看门狗 + 任务存活巡检 | ✅ | `housekeep_task.c`：IWDG 超时 1s、喂狗 200ms、`Health_Check` 巡检 |
| UWB 定位（BU03 / UART-AT + 车上本地解算） | ✅ | `bsp_uwb.c` + `uwb_task.c`：BU03（基于 DW3000）经 USART3 UART-AT 测距，主机 `UWB_Trilaterate2D` Gauss-Newton 三边定位解算坐标；坐标在车上本地算出，不依赖地面（注：仓库内 `Modules/dw1000/deca_device.c` 为 DW3000 官方驱动，未接入活动路径，故不适用 SPI CRC） |

**口径说明**：上表引用块为最终简历表述；本工程自述章节统一用「统一架构」表述，不出现「整合/迁移」字样。

### 九（补）量化数据支撑（简历可直接引用）

| 指标 | 数值 | 来源 |
|---|---|---|
| 目标 MCU | STM32F103RCT6（Cortex-M3 @72MHz，256KB Flash / 48KB RAM） | `board_config.h` |
| 固件体积（GNU 构建实测） | FLASH 47,984 B / 256 KB = **18.30%**；RAM 32,784 B / 48 KB = **66.70%** | `Build/firmware.map`（arm-none-eabi-gcc 13.x，零警告） |
| 任务调度 | FreeRTOS 10.5.1，6 任务，优先级 5/3/2/2/1/4 | `tasks.h` |
| 控制算法 | 串级双闭环 PID：外环姿态 PD（**部署值** Kp=5, Kd=2）+ 内环速度 PI 增量式（Kp=12, Ki=0.5）；Simulink 前期建模用外环 Kp=2, Kd=0.5 | `algo/pid.c`、`control_task.c`（宏）、`PID/Readme_PID.md` |
| 仿真性能 | 平直段零稳态误差 / 零超调；坡度 0.26 rad 阶跃 1.5s 内恢复（跌落 <15%）；风流扰动摆角 2.0s 收敛至 <0.02 rad | `PID/Readme_PID.md` |
| 安全机制 | 4 态状态机（RUN/WARN/BRAKE/ERROR，后两态锁定）；IWDG 超时 1s、喂狗周期 200ms、任务存活巡检 | `safety_fsm.h`、`housekeep_task.c` |
| 定位 | 集成安信可 BU03（基于 DW3000，UART-AT TWR），3 基站 Gauss-Newton 三边定位（周期 200ms、重试 3 次），坐标车上本地解算 | `uwb_task.c`、`bsp_uwb.c` |
| 双工具链 + 硬件烧录 | GCC：arm-none-eabi-gcc 产物 .elf/.hex/.bin 零警告（本机编译验证）；Keil ARMCC V5 + RVDS 移植层 + `syscalls.c` retarget 已修复，并**烧录至真实 STM32F103RCT6 板卡运行**（实机验证） | `build.py`、`stm32f103RCT6.uvprojx` |
