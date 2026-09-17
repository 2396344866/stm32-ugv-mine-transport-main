# 井下轨道运输车（单轨吊）分布式控制系统 — STM32F103RCT6 固件

> 基于 STM32F103RCT6 + FreeRTOS 10.5.1 的嵌入式固件，面向地下矿井轨道运输场景，
> 统一实现**运动控制 / 低功耗监测 / UWB 无线定位 / 车身 CAN 控制**四大功能，形成"感知—控制—定位—执行—回传地面监控"闭环。

## 一、核心特性

- **统一固件架构（RTOS 实时调度）**：以 STM32F103RCT6（Cortex-M3 @72MHz，256KB Flash / 48KB RAM）为主控，将运动控制、低功耗监测、UWB 定位统一为 FreeRTOS 10.5.1 多任务固件，按实时性划分为 7 个任务（控制 / 监测 / 定位 / 通信 / HMI / 看家 / 车身CAN，优先级 5/3/2/2/1/4/3），控制回路最高优先级抢占。
- **串级双闭环控制**：姿态外环 PD + 速度内环增量 PI（部署值 外环 Kp=5 / Kd=2，内环 Kp=12 / Ki=0.5；Simulink 前期建模用外环 Kp=2 / Kd=0.5）；基于 Simulink 完成建模与参数预整定（平直段零超调、坡度突变 1.5 s 恢复等动态指标），算法部署至 STM32F103RCT6 实板并搭建台架完成坡道运行验证，实机调速表现与仿真预期一致。
- **端侧高稳定性与安全机制**：4 态安全状态机（RUN / WARN / BRAKE / ERROR，后两态锁定）+ 独立看门狗（IWDG 超时 1 s、喂狗 200 ms）+ 任务存活巡检；迟滞报警与故障分级恢复。
- **UWB 定位**：集成安信可 BU03（基于 DW3000，UART-AT）TWR 测距，三基站 Gauss-Newton 三边定位（周期 200 ms、重试 3 次），坐标车上本地解算，无需地面参与。
- **双工具链**：GCC（arm-none-eabi-gcc）编译零警告，固件 FLASH 占用 18.3%（约 47 KB / 256 KB）；Keil ARMCC V5 工程已编译并烧录至实板运行验证。

## 二、软件架构 / 任务划分

| 任务 | 优先级 | 周期 | 职责 |
|---|---|---|---|
| 控制任务（control） | 5（最高） | 1 ms（TIM7 节拍） | 编码器测速 + MPU6050 倾角 + 串级 PID 输出 PWM + 电磁刹车 |
| 监测任务（monitor） | 3 | 50 ms | 压力变送器 ADC 混合滤波 + 迟滞报警 |
| 定位任务（uwb） | 2 | 200 ms | 集成安信可 BU03（基于 DW3000）TWR 测距 + 三边定位（坐标车上本地解算） |
| 通信任务（comm） | 2 | 1000 ms | 遥测帧经调试串口输出；ESP8266 WiFi 为可选备份上行 |
| HMI 任务（hmi） | 1 | 200 ms | OLED 刷新 + 按键扫描 |
| 看家任务（housekeep） | 4 | 200 ms | IWDG 喂狗 + 任务存活巡检 + 健康管理 |
| 车身任务（bcm） | 3 | 20 ms | 片内 bxCAN（PA11/PA12 + TJA1050，500 kbit/s）向车门控制器下发门锁指令；回执按 SEQ 配对，超时重发 ≤3 次 |

> 车身 CAN：报文的目标地址由 29 位扩展 ID 的 DA 位域承载，控制参数由数据场承载。
> 落锁帧示例 `ID 0x0601 2007` + `Data 0F 01 46 28`（四门 / 落锁 / 70% / 400 ms，DLC=4）。
> 位时基、过滤器、时序预算与两级验收方法详见 `Doc/CAN_BCM车门落锁.md`。

分层：`User/app`（algo/bsp/sys/tasks）→ `User/Modules`（mpu6050/dw1000）→ `Middlewares/FreeRTOS` → `FWLib`（StdPeriph）→ `Core/CMSIS` → `Startup`。

## 三、目录结构

```
RCT6/
├── Core/            CMSIS 内核（core_cm3 等）
├── FWLib/            STM32 标准外设库（StdPeriph）
├── Middlewares/     FreeRTOS 10.5.1（RVDS/ARM_CM3 移植层）
├── Startup/         Keil / GCC 启动文件
├── User/            应用层
│   ├── app/         algo(算法) / bsp(驱动) / sys(系统) / tasks(任务)
│   ├── Modules/     mpu6050(用户封装) / dw1000(休眠, 实为 DW3000 驱动)
│   ├── main.c       系统入口
│   ├── board_config.h  引脚与资源配置
│   └── syscalls.c   Keil/ARMCC retarget（printf → USART）
├── Project/         Keil 工程（stm32f103RCT6.uvprojx）+ GNU 构建脚本 build.py
├── PID仿真测参/      Simulink 串级双闭环 PID 模型与参数
├── Doc/             README（详细工程文档）
└── Readme.md        本文件
```

> 引脚与资源分配以 `User/board_config.h` 为唯一事实源，请以该文件为准。

## 四、硬件平台（要点）

- MCU：STM32F103RCT6（Cortex-M3，72 MHz，256 KB Flash / 48 KB RAM，HSE 8 MHz）
- 控制：增量式编码器（TIM2 正交解码）、MPU6050（I2C2 姿态）、TIM1 PWM 电机驱动、电磁刹车继电器
- 监测：扩散硅压力变送器（ADC1_CH4）→ 信号调理 → ADC；ESP8266-01S 可选备份上行；AT24C256 存储；OLED / LED / RTC / IWDG
- 定位：集成安信可 BU03（基于 DW3000，UART-AT，USART3）TWR 测距，坐标车上本地解算

## 五、构建与运行

### 5.1 GNU 工具链（验证用，零警告）
需 `arm-none-eabi-gcc`（如 STM32CubeCLT 自带）：
```bash
cd Project
python build.py
# 产物：Build/firmware.elf / .hex / .bin
```

### 5.2 Keil MDK（实板烧录）
1. 用 Keil 打开 `Project/stm32f103RCT6.uvprojx`
2. `Project → Options → C/C++` 已配置全局宏 `STM32F10X_HD, USE_STDPERIPH_DRIVER, MPU6050, HSE_VALUE=8000000`
3. 确保 **MicroLIB 关闭**（工程默认关闭）——本固件使用完整 C 库 + `_sys_*` retarget 以支持 `%f` 浮点打印
4. `F7` Rebuild → `F8` Download 烧录至 STM32F103RCT6 板卡

## 六、⚠️ 第三方库说明（重要）

本仓库**不包含**以下受许可证约束、禁止再分发的第三方库（本地编译所需，但不得随仓发布）：

- `Motion_driver/`（InvenSense / TDK 的 eMPL & MotionDriver 全套：mllite / mpl / driver / eMPL-hal 等）
- `User/Modules/mpu6050/` 中的 eMPL 文件（`inv_mpu.c/.h`、`inv_mpu_dmp_motion_driver.c/.h`、`dmpKey.h`、`dmpmap.h`、`libmpllib.lib`）

**若需从源码构建**，请从 TDK / InvenSense 官方获取 MotionDriver 包，按工程内的 DMP 移植指南放置上述文件后编译。本工程仅发布作者自研代码（`User/app`、`User/Modules/mpu6050` 中的用户封装部分、`board_config.h`、`syscalls.c` 等）。

## 七、许可

- 作者自研代码：本项目采用 [MIT](https://opensource.org/licenses/MIT) 许可。
- 第三方组件（FreeRTOS、STM32 StdPeriph、CMSIS、InvenSense MotionDriver 等）遵循各自原始许可证，不随本仓再分发。
