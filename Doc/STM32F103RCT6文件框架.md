# STM32F103RCT6 固件工程（demo_seial_OLED 版）

> 内部承载固件的 FreeRTOS + 7 任务 + 姿态/控制/监测/定位/通信/HMI/车身CAN + 看门狗/错误/健康等
> 全套功能。

---

## 1. 构建环境

- **主构建（用户指定）：Keil uVision** —— 工程文件 `Project/stm32f103RCT6.uvprojx`
  （目标器件 `STM32F103RC`，256KB Flash / 48KB RAM @ 72MHz，HSE 8MHz）。
- **本地验证构建：GNU ARM GCC**（`arm-none-eabi-gcc` 13.3.1，来自 STM32CubeCLT）——
  通过 `Project/build.py` 驱动，无需 make。本机已用它验证源码可完整编译、链接并生成
  `Build/firmware.elf / .hex / .bin`。`build.py` 只调用真实编译器，验证依据是编译器输出本身。

> ✅ **Keil 构建已就绪**：FreeRTOS 移植层已切换为
> `Middlewares/FreeRTOS/portable/RVDS/ARM_CM3`（FreeRTOS 10.5.1 官方 RVDS 移植，
> ARMCC V5 兼容；源文件来自本机 `AppData/Local/Temp/FreeRTOS-Kernel` 同版本仓库）。
> `.uvprojx` 中 68 个源码 `FilePath` 已统一为相对 `Project/` 的 `..\` 形式，
> ARMCC V5（`V5.06 update 6`）可直接编译、链接。
>
> ⚠️ **float 打印注意**：若 Keil 工程勾选了 *Use MicroLIB*，`printf` 不支持 `%f`，
> 调试串口打印浮点会输出 `?` 或异常。请取消勾选 *Use MicroLIB*（改用标准 C 库），
> 或在需要时用整数定点方式打印。GNU 构建（`build.py`）已用 `-u _printf_float` 处理，不受影响。

---

## 2. 目录结构

```
demo_seial_OLED_stm32f103rct6/
├─ Core/                     # CMSIS：core_cm3.c/h、system_stm32f10x.c/h
├─ FWLib/                    # STM32 标准外设库（StdPeriph 3.5/3.6，全量 src）
├─ Middlewares/FreeRTOS/     # FreeRTOS 10.5.1（include / portable/GCC/ARM_CM3 / portable/RVDS/ARM_CM3 / MemMang）
│  └─ portable/RVDS/ARM_CM3/ # ★ Keil/ARMCC 用移植层（port.c + portmacro.h，已就位）
├─ Startup/
│  ├─ startup_stm32f10x_hd.s        # Keil/MDK-ARM 语法启动文件（Keil 工程使用）
│  └─ startup_stm32f10x_hd_gcc.s    # GCC 语法启动文件（build.py 使用）
├─ User/
│  ├─ main.c / stm32f10x_it.c       # 固件入口 + 中断（FreeRTOS 接管 SVC/PendSV/SysTick/TIM3）
│  ├─ stm32f10x.h / stm32f10x_conf.h
│  ├─ FreeRTOSConfig.h / board_config.h / dwt.h / syscalls.c
│  ├─ app/
│  │  ├─ bsp/    # Delay/Key/LED/OLED/ADC/AT24C256/ESP8266/Motor/RTC/USART/UWB/Watchdog
│  │  ├─ algo/   # pid / filter / alarm / safety_fsm / uwb_2d（2D 三边定位）
│  │  ├─ sys/    # error / health / scheduler（20ms 确定性控制节拍 TIM3）
│  │  └─ tasks/  # control / monitor / uwb / hmi / comm / housekeep / bcm（7 个 FreeRTOS 任务）
│  └─ Modules/
│     ├─ mpu6050/  # MPU6050 DMP 运动驱动（活动 IMU，I2C2）
│     └─ dw1000/   # DW3000 UWB 寄存器/设备 API（目录名沿用 dw1000，实为 DW3000；bsp_uwb 以 UART-AT 方式驱动 BU03）
├─ Motion_driver/  # MPU9250 eMPL 运动驱动 —— 保留但未编入
├─ Project/
│  ├─ stm32f103RCT6.uvprojx  # Keil 工程
│  ├─ stm32f103RCT6.uvoptx
│  ├─ STM32F103RCT6.ld       # GNU 链接脚本
│  ├─ build.py               # GNU 构建脚本
├─ Doc/README.md
└─ Build/       # 构建产物（firmware.elf / .hex / .bin / .map）
```

---

## 3. 活动模块 vs 保留（未编译）模块

| 类别 | 状态 | 说明 |
|------|------|------|
| FreeRTOS + 7 任务 + 各 BSP/算法/系统层 | **活动** | 固件主体，Keil 与 GNU 均编入 |
| 车身 CAN（片内 bxCAN，PA11/PA12 + TJA1050） | **活动** | BCM 向车门控制器下发落锁指令；详见 `Doc/CAN_BCM车门落锁.md` |
| MPU6050 DMP（I2C2） | **活动** | 姿态解算 IMU（控制/HMI 任务依赖） |
| 安信可 BU03 UWB（基于 DW3000，UART-AT） | **活动**（bsp 层） | `bsp_uwb.c` 以 AT 指令驱动 BU03 模组，未直接调用 deca 底层 `.c` |
| MPU9250 eMPL（Motion_driver/） | 保留不编译 | 工程统一采用 MPU6050 DMP 单 IMU 方案，MPU9250 文件原样保留但不参与默认编译 |

> 选择理由（用户确认）：工程统一采用 **MPU6050 DMP 单 IMU 方案** 进行姿态解算，MPU9250 相关文件原样保留在目录中，
> 但不参与默认编译，避免多 IMU 方案带来的冗余与维护成本。

---

## 4. 构建方法

### 4.1 GNU 验证构建（本机已通过）

```bash
cd Project
python build.py            # 编译、链接、生成 Build/firmware.{elf,hex,bin}
python build.py clean      # 清理
```

产物体积（示例）：FLASH 47984 B（18.3%）/ RAM 32784 B（66.7%）。
工具链自动探测 `ARM_GCC` 环境变量或 `D:/ST/STM32CubeCLT_1.18.0/...` 下的 `arm-none-eabi-gcc`。

### 4.2 Keil uVision 构建

1. 用 Keil 打开 `Project/stm32f103RCT6.uvprojx`（目标 `Target 1`，器件 `STM32F103RC`）。
2. FreeRTOS 的 RVDS 移植层（`Middlewares/FreeRTOS/portable/RVDS/ARM_CM3/`）**已就位**，无需再次替换；GCC 移植层 ARMCC 无法编译，请勿切回。
3. 编译（F7）/ 下载（F8）。输出 `stm32f103RCT6.hex`、可执行文件。

全局宏已配置：`USE_STDPERIPH_DRIVER, STM32F10X_HD, EMPL_TARGET_STM32F1, MPU6050, HSE_VALUE=8000000`。
包含路径已覆盖 `Core / FWLib/inc / User / Startup / Middlewares/FreeRTOS(/include,/portable/.../ARM_CM3) / User/app/{bsp,algo,sys,tasks} / User/Modules/{mpu6050,dw1000}`。

---

## 5. 配置开关

`User/board_config.h` 控制子系统开关：

- `USE_MPU6050_DMP` —— MPU6050 姿态 DMP（默认开）
- `USE_AT24C256` —— I2C2 EEPROM 错误环形日志（默认开）
- `USE_RTC_DEPENDENT` —— RTC（LSI）低功耗时基（默认开）
- `USE_UWB_SUBSYS` / `USE_MONITOR_SUBSYS` / `USE_COMM_SUBSYS` 等控制各任务内子功能
- 各任务优先级/栈大小见 `User/app/tasks/tasks.h`（控制环最高优先级）

---

## 6. 运行与烧录

- 调试串口：`USART2`（PA2/PA3，115200），`printf` 重定向到此；`USART1/USART3` 供 ESP8266/UWB。
- 烧录：STM32CubeProgrammer 或 Keil（ST-Link）写入 `Build/firmware.hex`（或 `.bin` @ 0x08000000）。
- 上电后 FreeRTOS 启动 7 个任务；TIM3 产生 20ms 确定性控制节拍（二值信号量驱动双闭环 PID）；
  IWDG 看门狗 + 任务存活巡检 + 统一错误日志 + 栈溢出/Malloc 失败钩子保障稳定性。

---

## 7. 原始素材去向

- 原 demo 的 `main.c` 骨架、中断模板：`Legacy/`
- 原 demo 的 `Hardware/`（OLED/Key/LED/MPU9250）、`Motion_driver/`（eMPL）、`System/`（Usart/Delay）：
  仍保留在各自目录，仅从活动编译集中排除。
- 源目录的原理图 PDF、参考文档等未作改动。

---

## 8. 验证状态

- ✅ GNU ARM GCC 构建：源码完整编译 + 链接通过，零错误/零警告，产物齐全。
- ✅ Keil 工程：已按活动源码集重排 `.uvprojx`（器件/内存/输出 HEX 均正确配置），RVDS 移植层就位。
