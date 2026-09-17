#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32F103RCT6 固件构建脚本（不依赖 make）
直接调用 arm-none-eabi-gcc / objcopy / size 完成编译-链接-格式转换。

本项目 = demo_seial_OLED_stm32f103rct6 整合工程：
  - 整合了 STM32F103RCT6_IntegratedFirmware 的全部内容（FreeRTOS + 6 任务 +
    姿态/控制/监测/定位/通信/HMI + MPU6050 DMP + UWB + 看门狗/错误/健康等）
  - 原 demo 的串行+OLED+MPU9250 源码保留在 Hardware/ Motion_driver/ System/
    （默认不编入，避免双 IMU/双驱动符号冲突；MPU6050 为活动 IMU）

注意：本脚本仅用于在本机用 GNU 工具链验证“源码可编译/可链接”。
最终交付以 Keil uVision 工程（Project/stm32f103RCT6.uvprojx）为准，
Keil 下请把 FreeRTOS 移植层切换为 Middlewares/FreeRTOS/portable/RVDS/ARM_CM3
（GCC 版移植层的内联汇编 ARMCC 不兼容）。

用法:
    python build.py            # 编译、链接、生成 .elf/.hex/.bin
    python build.py clean      # 清理 Build/ 产物
"""
import os
import sys
import glob
import subprocess
import shutil

# ------------------------- 路径与工具链 -------------------------
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 工具链：优先环境变量 ARM_GCC，否则尝试常见安装位置
TOOLCHAIN_CANDIDATES = [
    os.environ.get("ARM_GCC", ""),
    r"D:/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe",
    r"C:/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe",
    "/d/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe",
]
GCC = None
for c in TOOLCHAIN_CANDIDATES:
    if c and os.path.isfile(c):
        GCC = c
        break
if GCC is None:
    print("[ERROR] 未找到 arm-none-eabi-gcc，请设置环境变量 ARM_GCC 指向该可执行文件。")
    sys.exit(1)
TOOLBIN = os.path.dirname(GCC)
OBJCOPY = os.path.join(TOOLBIN, "arm-none-eabi-objcopy.exe")
SIZE    = os.path.join(TOOLBIN, "arm-none-eabi-size.exe")

BUILD = os.path.join(ROOT, "Build")
OBJDIR = os.path.join(BUILD, "obj")
LINKER = os.path.join(ROOT, "Project", "STM32F103RCT6.ld")
ELF = os.path.join(BUILD, "firmware.elf")
HEX = os.path.join(BUILD, "firmware.hex")
BIN = os.path.join(BUILD, "firmware.bin")
MAP = os.path.join(BUILD, "firmware.map")

# ------------------------- 编译参数 -------------------------
INCLUDES = [
    "Core",                                   # core_cm3.h / system_stm32f10x.h
    "FWLib/inc",                              # STM32 标准外设库头
    "Middlewares/FreeRTOS/include",
    "Middlewares/FreeRTOS/portable/GCC/ARM_CM3",
    "User",                                   # stm32f10x.h（CMSIS 器件头）
    "User/app/bsp",
    "User/app/algo",
    "User/app/sys",
    "User/app/tasks",
    "User/Modules/mpu6050",
    "User/Modules/dw1000",
]
INC_FLAGS = ["-I" + os.path.join(ROOT, p) for p in INCLUDES]

DEFINES = [
    "STM32F10X_HD",
    "USE_STDPERIPH_DRIVER",
    "EMPL_TARGET_STM32F1",
    "MPU6050",
    "HSE_VALUE=8000000",
]

CFLAGS = [
    "-mcpu=cortex-m3", "-mthumb",
    "-specs=nano.specs", "-u", "_printf_float",
    "-ffunction-sections", "-fdata-sections",
    "-O2", "-g", "-Wall",
    "-Wno-unused-parameter", "-Wno-unused-variable",
    "-Wno-strict-aliasing",
]

# ------------------------- 源文件清单 -------------------------
SOURCES = [
    # CMSIS（GCC 语法启动文件）
    "Core/core_cm3.c",
    "Core/system_stm32f10x.c",
    "Startup/startup_stm32f10x_hd_gcc.s",
    # 标准外设库（全量）
] + sorted(glob.glob(os.path.join(ROOT, "FWLib/src/*.c"))) + [
    # FreeRTOS 内核
    "Middlewares/FreeRTOS/croutine.c",
    "Middlewares/FreeRTOS/event_groups.c",
    "Middlewares/FreeRTOS/list.c",
    "Middlewares/FreeRTOS/queue.c",
    "Middlewares/FreeRTOS/stream_buffer.c",
    "Middlewares/FreeRTOS/tasks.c",
    "Middlewares/FreeRTOS/timers.c",
    "Middlewares/FreeRTOS/portable/GCC/ARM_CM3/port.c",
    "Middlewares/FreeRTOS/portable/MemMang/heap_4.c",
    # 用户入口与系统层
    "User/main.c",
    "User/syscalls.c",
    "User/stm32f10x_it.c",
    "User/app/sys/error.c",
    "User/app/sys/health.c",
    "User/app/sys/scheduler.c",
    # 算法层
    "User/app/algo/pid.c",
    "User/app/algo/filter.c",
    "User/app/algo/alarm.c",
    "User/app/algo/safety_fsm.c",
    "User/app/algo/uwb_2d.c",
    "User/app/algo/bcm_door.c",
    # BSP 层
    "User/app/bsp/Delay.c",
    "User/app/bsp/Key.c",
    "User/app/bsp/LED.c",
    "User/app/bsp/oled.c",
    "User/app/bsp/bsp_adc.c",
    "User/app/bsp/bsp_at24c256.c",
    "User/app/bsp/bsp_esp8266.c",
    "User/app/bsp/bsp_motor.c",
    "User/app/bsp/bsp_rtc.c",
    "User/app/bsp/bsp_usart.c",
    "User/app/bsp/bsp_uwb.c",
    "User/app/bsp/bsp_watchdog.c",
    "User/app/bsp/bsp_can.c",
    # 任务层
    "User/app/tasks/control_task.c",
    "User/app/tasks/monitor_task.c",
    "User/app/tasks/uwb_task.c",
    "User/app/tasks/hmi_task.c",
    "User/app/tasks/comm_task.c",
    "User/app/tasks/housekeep_task.c",
    "User/app/tasks/bcm_task.c",
    # MPU6050 DMP 运动驱动
    "User/Modules/mpu6050/MPU6050.c",
    "User/Modules/mpu6050/inv_mpu.c",
    "User/Modules/mpu6050/inv_mpu_dmp_motion_driver.c",
    "User/Modules/mpu6050/mpu_port.c",
]
SOURCES = [os.path.join(ROOT, s) for s in SOURCES]

LDFLAGS = [
    "-mcpu=cortex-m3", "-mthumb",
    "-specs=nano.specs",
    "-T", LINKER,
    "-Wl,--gc-sections",
    "-Wl,-Map=" + MAP,
    "-Wl,--print-memory-usage",
]

# ------------------------- 工具函数 -------------------------
def run(cmd):
    print(">> " + os.path.basename(cmd[-1]), flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.stdout:
        sys.stdout.write(r.stdout)
    if r.stderr:
        sys.stderr.write(r.stderr)
    if r.returncode != 0:
        print("[FAIL] 命令返回非零: " + " ".join(cmd))
        sys.exit(r.returncode)
    return r

def clean():
    if os.path.isdir(BUILD):
        shutil.rmtree(BUILD)
    print("[OK] 已清理 Build/")

def obj_path(src):
    base = os.path.basename(src)
    if src.endswith(".s"):
        base = base[:-2] + ".o"
    else:
        base = base[:-2] + ".o"
    return os.path.join(OBJDIR, base)

def build():
    os.makedirs(OBJDIR, exist_ok=True)
    objs = []
    for src in SOURCES:
        if not os.path.isfile(src):
            print("[WARN] 源文件缺失，跳过: " + src)
            continue
        o = obj_path(src)
        objs.append(o)
        # 增量编译：目标文件比源文件新则跳过
        if os.path.isfile(o) and os.path.getmtime(o) >= os.path.getmtime(src):
            continue
        cmd = [GCC] + CFLAGS + INC_FLAGS
        for d in DEFINES:
            cmd.append("-D" + d)
        cmd += ["-c", src, "-o", o]
        run(cmd)

    # 链接（libm 必须在目标文件之后，静态库按引用解析）
    cmd = [GCC] + LDFLAGS + objs + ["-lm", "-o", ELF]
    run(cmd)

    # 格式转换
    run([OBJCOPY, "-O", "ihex", ELF, HEX])
    run([OBJCOPY, "-O", "binary", ELF, BIN])

    # 体积报告
    if os.path.isfile(SIZE):
        run([SIZE, ELF])

    print("\n[BUILD OK]")
    print("  ELF : " + ELF)
    print("  HEX : " + HEX)
    print("  BIN : " + BIN)
    print("  MAP : " + MAP)

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "clean":
        clean()
    else:
        build()
