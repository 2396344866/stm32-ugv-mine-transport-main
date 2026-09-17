#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN 子系统改动 —— 本机编译/链接验证脚本（不属于固件工程，位于临时目录）

做法：
  1. 直接复用 Project/build.py 的编译器、编译选项、宏定义与包含路径（保持一致）；
  2. 用 verify/mpu_stub.c 替换缺失第三方依赖的 User/Modules/mpu6050/MPU6050.c
     （仓库按合规要求排除了 Motion_driver eMPL 源码，属既有基线问题）；
  3. 完整编译 + 链接，验证新增的 bsp_can.c / bcm_door.c / bcm_task.c
     及其对工程其余部分的改动（tasks.h / error.c / stm32f10x_it.c / main.c）
     可以无错通过并可链接成完整固件。
"""
import os, sys, subprocess, importlib.util

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
VERIFY = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(VERIFY, "build")
OBJDIR = os.path.join(BUILD, "obj")

# 复用 build.py 的工具链/flags（以 __name__ != "__main__" 载入，不会触发构建）
spec = importlib.util.spec_from_file_location("fwbuild", os.path.join(ROOT, "Project", "build.py"))
fw = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fw)

GCC     = fw.GCC
OBJCOPY = fw.OBJCOPY
SIZE    = fw.SIZE
LINKER  = fw.LINKER

sources = [s for s in fw.SOURCES
           if not os.path.normpath(s).replace("\\", "/").endswith("mpu6050/MPU6050.c")]
sources.append(os.path.join(VERIFY, "mpu_stub.c"))
assert not any("mpu6050/MPU6050.c" in os.path.normpath(s).replace("\\", "/") for s in sources)

STUB_OBJ = os.path.join(OBJDIR, "mpu_stub.o")

# 新增/改动的源码单独加强告警等级审查
STRICT_FILES = ["bsp_can.c", "bcm_door.c", "bcm_task.c", "error.c",
                "stm32f10x_it.c", "main.c", "control_task.c", "hmi_task.c"]


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.stdout: sys.stdout.write(r.stdout)
    if r.stderr: sys.stderr.write(r.stderr)
    if r.returncode != 0:
        print("[FAIL] " + " ".join(cmd[:6]) + " ...")
        sys.exit(r.returncode)


os.makedirs(OBJDIR, exist_ok=True)
objs = []
warn_strict = 0

for src in sources:
    if not os.path.isfile(src):
        print("[WARN] 缺失跳过: " + src)
        continue
    base = os.path.basename(src)[:-2] + ".o"
    o = os.path.join(OBJDIR, base)
    objs.append(o)
    cmd = [GCC] + fw.CFLAGS + fw.INC_FLAGS + ["-D" + d for d in fw.DEFINES]
    if base[:-2] in STRICT_FILES or base in STRICT_FILES:
        cmd += ["-Wextra", "-Wshadow", "-Wno-unused-parameter"]
    cmd += ["-c", src, "-o", o]
    print(">> " + os.path.basename(src))
    r = subprocess.run(cmd, capture_output=True, text=True)
    out = (r.stdout or "") + (r.stderr or "")
    if base in ("bsp_can.o", "bcm_door.o", "bcm_task.o"):
        if out.strip():
            print(out)
            warn_strict += out.count("warning:")
    if r.returncode != 0:
        print(out)
        print("[FAIL] 编译失败: " + src)
        sys.exit(r.returncode)

ELF = os.path.join(BUILD, "verify.elf")
run([GCC] + fw.LDFLAGS + ["-Wl,-Map=" + os.path.join(BUILD, "verify.map")] + objs + ["-lm", "-o", ELF])
run([OBJCOPY, "-O", "ihex", ELF, os.path.join(BUILD, "verify.hex")])
run([SIZE, ELF])

print()
print("[VERIFY OK] 新增 CAN 源码编译 + 链接通过")
print("  严格告警文件数: %d, 新模块告警数: %d" % (len(STRICT_FILES), warn_strict))

# ------------------------- 回环自检分支也参与编译 -------------------------
# 默认 CAN_LOOPBACK_SELFTEST=0，该分支是死代码、永不被编译 = 永不被验证。
# 这里强制以 -DCAN_LOOPBACK_SELFTEST=1 再编译一遍 two files，把它拉进视野。
print("\n[+] 回环自检分支编译检查(-DCAN_LOOPBACK_SELFTEST=1)")
for src in (os.path.join(ROOT, "User", "app", "bsp", "bsp_can.c"),
            os.path.join(ROOT, "User", "app", "tasks", "bcm_task.c")):
    cmd = [GCC] + fw.CFLAGS + fw.INC_FLAGS + ["-D" + d for d in fw.DEFINES] + [
        "-Wextra", "-Wshadow", "-Wno-unused-parameter",
        "-DCAN_LOOPBACK_SELFTEST=1", "-c", src,
        "-o", os.path.join(OBJDIR, "loopback_" + os.path.basename(src)[:-2] + ".o")]
    print(">> loopback_" + os.path.basename(src))
    run(cmd)
print("[OK] 回环分支编译通过（两遍都清理，说明两条配置路径均无语法/类型错误）")

# ------------------------- 协议层主机单元测试 -------------------------
# 用 PC 编译器把协议层真跑一遍：把 Doc 里写死的实例帧反过来拴住代码实现。
HOST_GCC = None
for c in (os.environ.get("HOST_GCC", ""), r"C:/Strawberry/c/bin/gcc.exe", "gcc"):
    if not c:
        continue
    if os.path.isfile(c) or subprocess.run(["where" if os.name == "nt" else "which", c],
                                           capture_output=True).returncode == 0:
        HOST_GCC = c
        break

TP = os.path.join(VERIFY, "test_protocol")
TEST = os.path.join(TP, "test_bcm_door.c")
TEST_BIN = os.path.join(BUILD, "bcm_door_test.exe")

if HOST_GCC and os.path.isfile(TEST):
    print("\n[+] 协议层主机单元测试（PC 真实运行）")
    run([HOST_GCC, "-Wall", "-Wextra", "-std=c99",
         "-I" + TP, "-I" + os.path.join(ROOT, "User", "app", "algo"),
         TEST, os.path.join(ROOT, "User", "app", "algo", "bcm_door.c"),
         "-o", TEST_BIN])
    run([TEST_BIN])
else:
    print("\n[SKIP] 未找到 PC 编译器或无测试用例，跳过协议层单元测试")
