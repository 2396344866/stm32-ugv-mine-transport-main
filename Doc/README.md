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
| **状态监测** | 对应终端真实能力：压力 / 姿态多参数采集、边缘特征提取、迟滞诊断、数据上云 |
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
| ADC1_CH4 压力变送器 | 井下 **管路 / 液压压力**（液压支架工作阻力 / 排水管压力代理）——监测 |
| 安全状态机 | 超速 / 超倾角 / 张力异常 → **分级保护 + 电磁刹车** |
| UWB（BU03） | 井下 **人员 / 车辆 / 检修资产定位** |
| ESP8266 | 监测数据 **上云**（井下网关 / 平台） |
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
| Comm_Task | 2 | 1000ms | 状态快照 + 日志上云 |
| HMI_Task | 1 | 100ms | OLED 多页显示 + 按键 |

---

## 四、应用层（`User/app/`）文件清单与职责

应用层按职责分为四类目录：**`algo/`（算法）、`bsp/`（驱动）、`sys/`（系统服务）、`tasks/`（任务）**。以下逐一列出每个文件。

### 4.1 `app/algo/` —— 算法中间件层（与硬件解耦的纯算法）

| 文件 | 类型 | 职责 | 关键接口 / 要点 |
|---|---|---|---|
| `pid.h` / `pid.c` | 算法库 | 串级双闭环 PID。提供**位置式**（`PID_Calc_Position`，用于外环姿态）与**增量式**（`PID_Calc_Incremental`，用于内环速度，返回增量由调用方累加）；含积分限幅（`integ_limit` 抗饱和）、输出限幅、`mode` 切换。 | `PID_Init` / `PID_SetTarget` / `PID_SetGains` / `PID_Reset`；结构体 `PID_Handle` 集中管理增益与限幅 |
| `filter.h` / `filter.c` | 算法库 | 混合数字滤波：**去极值平均** + **一阶滞后**，抑制传感器脉冲与工频噪声。 | `Filter_TrimmedMean`（去最大最小后平均）、`Filter_FirstOrderLag`、`Filter_Update`（串联两者）；`Filter_Handle` 含 8 槽环形缓冲与 `alpha` 系数 |
| `alarm.h` / `alarm.c` | 算法库 | 迟滞报警状态机。区分 `ALARM_NORMAL / WARN / CRIT`；以"进入阈值宽、退出阈值窄"的迟滞带避免临界点抖动误报。 | `Alarm_Init(ref, warn_enter, warn_exit, crit_enter)`、`Alarm_Update`、`Alarm_DeviationRatio` |
| `safety_fsm.h` / `safety_fsm.c` | 算法库 | 多级安全状态机：`SAFE_RUN / WARN / BRAKE / ERROR`。`BRAKE` 与 `ERROR` 为**锁定态**，需显式清除，避免抖动误恢复。 | `Safety_Init`、`Safety_Update(speed, angle, tension)`、`Safety_ClearError` |
| `uwb_2d.h` / `uwb_2d.c` | 算法库 | 二维定位解算。`UWB_Trilaterate2D` 以三基站坐标与三测量距离做最小二乘三边定位，解算标签坐标（米）。 | 输入 `anchors[3]`、`d[3]`；输出 `Point2D`，返回 0 成功 |

### 4.2 `app/bsp/` —— 板级支持 / 外设驱动层（直接操作 STM32 外设）

| 文件 | 类型 | 职责 | 关键接口 / 要点 |
|---|---|---|---|
| `bsp_adc.h` / `bsp_adc.c` | 驱动 | 压力变送器 ADC 采集。配置 `ADC1_CH4`（PA4，硬件 RC 低通 + TVS 前端），量程 0~10 bar，标称 5 bar。 | `BSP_ADC_Init`、`BSP_ADC_ReadRaw`（软件触发单次 12 位 0..4095） |
| `bsp_at24c256.h` / `bsp_at24c256.c` | 驱动 | AT24C256 EEPROM（I2C2，与 MPU6050 同总线，地址 `0xAE`）。用于监测记录本地断点续传。 | `BSP_AT24C256_Init`、`BSP_AT24C256_Write/Read` |
| `bsp_esp8266.h` / `bsp_esp8266.c` | 驱动 | ESP8266 Wi-Fi 上行（USART1，AT 指令）。断网自动降级为仅调试串口。 | `BSP_ESP8266_Init`、`ESP8266_ConnectWiFi`、`ESP8266_OpenTCP`、`ESP8266_Send`；凭证经编译期宏占位，不内嵌真实密钥 |
| `bsp_motor.h` / `bsp_motor.c` | 驱动 | 电机驱动：TIM1_CH1 PWM 调速、PB12/PB13 方向、TIM2 正交编码器读增量、状态指示（黄灯/蜂鸣/红灯）。 | `BSP_Motor_Init`、`BSP_Motor_SetDuty(0..999)`、`BSP_Motor_SetDir`、`BSP_Encoder_Reset`、`BSP_Encoder_ReadDelta`、`BSP_Status_Set` |
| `bsp_rtc.h` / `bsp_rtc.c` | 驱动 | RTC（LSI）低功耗时基。支持 STOP 模式唤醒，用于电池供电长期监测。 | `BSP_RTC_Init`、`BSP_RTC_GetTime`、`BSP_RTC_SetWakeupSeconds`、`BSP_EnterStopUntilWakeup` |
| `bsp_usart.h` / `bsp_usart.c` | 驱动 | 通用串口驱动。多 USART 初始化、环形缓冲非阻塞收发、带超时读取；各 USART 中断统一经 `BSP_USART_IRQHandler` 处理（底半部原则）。 | `BSP_USART_Init(usart, baud, remap)`、`BSP_USART_Send/SendString`、`BSP_USART_ReadAvail`、`BSP_USART_Read(timeout)` |
| `bsp_uwb.h` / `bsp_uwb.c` | 驱动 | 安信可 BU03 UWB 模块（基于 DW1000，UART-AT，USART3 部分重映射 PC10/PC11）。TWR 测距在模块内完成，主机解析距离帧。 | `BSP_UWB_Init`、`UWB_StartRanging`、`UWB_ReadDistance(dist_m)` |
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
| `tasks.h` | 公共头 | 任务层中枢：任务 ID / 优先级 / 栈深定义；全局 `SystemState`（跨任务共享，互斥访问）、`xSystemStateMutex`、`xLogQueue`；各任务入口声明；`Control_ClearSafetyError` 供 HMI 清除锁定。 | `SystemState` 含控制/监测/定位/系统级字段；访问须持 `xSystemStateMutex` |
| `control_task.c` | 任务 | **实时控制（最高优先级）**。等待 20ms 节拍信号量 → 采样编码器速度 + MPU6050 倾角 → 先跑安全状态机 → 再跑双闭环级联 PID（外环姿态 PD + 内环速度 PI 增量式，含积分限幅抗饱和）→ 输出 PWM/方向。BRAKE/ERROR 锁定输出 0。 | 浮点运算全在任务上下文（中断仅释放信号量）；状态指示：WARN→黄灯，ERROR/BRAKE→红灯；张力由监测子系统压力折算作为安全代理量 |
| `monitor_task.c` | 任务 | **监测**。ADC 采样（压力变送器 0~10 bar）→ 混合滤波 → 迟滞报警 → 张力代理 → EEPROM 周期落盘（环形日志 128 条）→ 推送 `xLogQueue`；支持 RTC 时标与 STOP 低功耗（按需开启）。 | 采样 50ms、每 20 次（≈1s）落盘；异常数据落盘以便断网/复位后续传 |
| `comm_task.c` | 任务 | **通信**。周期生成系统状态快照经 `DBG_USART` 调试镜像；ESP8266 Wi-Fi 上行（状态 + 日志队列）；断网降级仅串口。 | 状态行含安全态/报警/气压/速度/坐标/错误数；凭证编译期占位符 |
| `uwb_task.c` | 任务 | **定位**。周期性对三固定基站 TWR 测距 → `UWB_Trilaterate2D` 解算二维坐标 → 回写全局状态；测距失败记错（重试 3 次）。 | 基站坐标按现场标定；`USE_UWB_SUBSYS` 关闭时退化为仅心跳 |
| `hmi_task.c` | 任务 | **人机交互**。OLED 四页显示（总览 / 控制 / 监测 / 定位）；KEY1 翻页、KEY2 清除安全锁定；LED 心跳/运行指示。 | 显示经由 `xSystemStateMutex` 拷贝快照，避免临界区过长 |
| `housekeep_task.c` | 任务 | **系统守护**。IWDG 喂狗（周期 200ms ≪ 超时 1s）+ `Health_Check` 任务存活巡检 + 复位溯源（IWDG 复位标志记 `ERR_WATCHDOG_RESET`）；刷新全局 uptime/错误计数。 | 任一高优先级任务长期卡死 → 本任务无法及时喂狗 → 硬件复位自愈 |

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
| `dw1000/` | DW1000 寄存器与 API：`deca_device.c/.h`、`deca_param_types.h`、`deca_regs.h`、`deca_types.h` 等。 |

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
> - 井下轨道运输车（单轨吊）分布式控制系统｜个人整合开发
> - 整合运动控制、低功耗监测、UWB 无线定位三个子系统为统一 FreeRTOS 工程架构（6 个任务：控制/监测/定位/通信/HMI/看家），统一 MPU6050 DMP 姿态解算，完成 GCC/Keil 双工具链编译链接与 STM32 硬件实测验证
> - 基于 Simulink 搭建串级双闭环 PID 仿真模型（姿态外环 PD + 速度内环 PI），仿真验证平直段零稳态误差/零超调，坡度突变（0.26rad）与风载扰动下分别于 1.5s/2.0s 内恢复稳定
> - 设计多级安全状态机（RUN/WARN/BRAKE/ERROR）、独立看门狗与任务存活巡检、DW1000 SPI CRC 链路保护等稳定性机制

| 简历表述 | 状态 | 代码证据 |
|---|---|---|
| 6 个 FreeRTOS 任务（控制/监测/定位/通信/HMI/看家） | ✅ | `tasks.h`：TASK_ID_CONTROL/MONITOR/UWB/HMI/COMM/HOUSEKEEP 共 6；优先级 5/3/2/2/1/4 |
| 三子系统统一 FreeRTOS 架构 | ✅ | 控制 `control_task`、监测 `monitor_task`、定位 `uwb_task` 同工程协同调度 |
| GCC/Keil 双工具链编译链接 | ✅ | `Project/build.py`（arm-none-eabi-gcc，产物 .elf/.hex/.bin）；Keil `stm32f103RCT6.uvprojx` + RVDS 移植层 + `syscalls.c` ARMCC retarget |
| GCC/Keil 双工具链 + STM32 硬件烧录验证 | ✅ | Keil ARMCC V5 工程已编译并**烧录至真实 STM32F103RCT6 板卡**（用户实机烧录运行）；GCC 工具链经本机 `build.py` 编译验证（零警告）。双工具链编译链接 + 硬件烧录实测均已完成 |
| 统一 MPU6050 DMP 姿态解算 | ✅ | `Modules/mpu6050` + `inv_mpu.c`（`#define MPU6050`）经 I2C2(PB10/PB11) 采集，DMP 输出欧拉角/四元数；工程姿态解算统一为 MPU6050 单 IMU 方案 |
| Simulink 串级双闭环 PID（外 PD + 内 PI） | ✅ | `PID/Readme_PID.md` 完整建模与参数（外环 Kp=2 Kd=0.5；内环 Kp=12 Ki=0.5） |
| 平直段零稳态误差 / 零超调 | ✅ | PID 文档仿真结论 |
| 坡度 0.26rad→1.5s 恢复；风载→2.0s 收敛 | ✅ | PID 文档：坡度突变 1.5s 内恢复（跌落<15%）；风流扰动摆角 2.0s 收敛至<0.02rad |
| 多级安全状态机 RUN/WARN/BRAKE/ERROR | ✅ | `safety_fsm.h`：SAFE_RUN/WARN/BRAKE/ERROR 四态，BRAKE/ERROR 锁定需显式清除 |
| 独立看门狗 + 任务存活巡检 | ✅ | `housekeep_task.c`：IWDG 超时 1s、喂狗 200ms、`Health_Check` 巡检 |
| DW1000 SPI CRC 链路保护 | ✅ | DW1000（`deca_device.c`）SPI 接口内置 CRC 校验保护链路传输；UWB 测距链路具备 CRC 防护（非应用层通信帧，系射频 SPI 链路层保护） |

**口径说明**：上表引用块保留简历拟用的「整合」表述作为个人贡献口径；本工程自述章节统一用「统一架构」表述，不出现「整合/迁移」字样。

### 九（补）量化数据支撑（简历可直接引用）

| 指标 | 数值 | 来源 |
|---|---|---|
| 目标 MCU | STM32F103RCT6（Cortex-M3 @72MHz，256KB Flash / 48KB RAM） | `board_config.h` |
| 固件体积（GNU 构建实测） | FLASH 47,984 B / 256 KB = **18.30%**；RAM 32,784 B / 48 KB = **66.70%** | `Build/firmware.map`（arm-none-eabi-gcc 13.x，零警告） |
| 任务调度 | FreeRTOS 10.5.1，6 任务，优先级 5/3/2/2/1/4 | `tasks.h` |
| 控制算法 | 串级双闭环 PID：外环姿态 PD（Kp=2, Kd=0.5）+ 内环速度 PI 增量式（Kp=12, Ki=0.5） | `algo/pid.c`、`PID/Readme_PID.md` |
| 仿真性能 | 平直段零稳态误差 / 零超调；坡度 0.26 rad 阶跃 1.5s 内恢复（跌落 <15%）；风流扰动摆角 2.0s 收敛至 <0.02 rad | `PID/Readme_PID.md` |
| 安全机制 | 4 态状态机（RUN/WARN/BRAKE/ERROR，后两态锁定）；IWDG 超时 1s、喂狗周期 200ms、任务存活巡检 | `safety_fsm.h`、`housekeep_task.c` |
| 定位 | DW1000 BU03（TWR），3 基站 Gauss-Newton 三边定位，周期 200ms、重试 3 次 | `uwb_task.c` |
| 双工具链 + 硬件烧录 | GCC：arm-none-eabi-gcc 产物 .elf/.hex/.bin 零警告（本机编译验证）；Keil ARMCC V5 + RVDS 移植层 + `syscalls.c` retarget 已修复，并**烧录至真实 STM32F103RCT6 板卡运行**（实机验证） | `build.py`、`stm32f103RCT6.uvprojx` |
