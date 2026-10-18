#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_task.c 的 EEPROM 日志布局 / CRC / 磨损寿命自校验。

布局参数一律**从源码宏解析**，不在此硬编码，避免代码改了就��校验脚本失配
（历史教训：曾把 12 B 记录的期望值写死，代码改成 16 B 后脚本一直报假 FAIL）。

磨损寿命依据 AT24C256C datasheet Table 4-6：
  Write Endurance = 1,000,000 Write Cycles，**按字节(cell)计，不按页计**。
  页(64B)只是 I2C 写缓冲的传输粒度，擦写只作用到目标字节的浮栅。
  ⇒ head A / head B 分处两个字节单元，A/B 交替即得2× 延寿；
    **无需为「跨页」挪地址**（收益为零，且会讲错 EEPROM 机理）。
"""
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "User", "app", "tasks", "monitor_task.c")

ENDURANCE = 1_000_000
PAGE = 64
text = open(SRC, encoding="utf-8").read()


def macro(name):
    """从 monitor_task.c 解析 #define <name> (0x....) 或 (数字)"""
    m = re.search(r"^#define\s+%s\s+\(?\s*(0x[0-9A-Fa-f]+|\d+)" % name, text, re.M)
    if not m:
        raise SystemExit("[FAIL] 源码中找不到宏 %s" % name)
    return int(m.group(1), 0)


BASE = macro("EEPROM_LOG_BASE")
N = macro("EEPROM_LOG_N")
RECLEN = macro("EEPROM_LOG_RECLEN")
CRC_OFF = macro("EEPROM_REC_CRC_OFF")
HEAD_A = macro("EEPROM_HEAD_A_ADDR")
HEAD_B = macro("EEPROM_HEAD_B_ADDR")
LOG_PERIOD = macro("MONITOR_LOG_PERIOD")
RING_N = macro("MONITOR_RING_N")
SAMPLE_MS = macro("MONITOR_PERIOD_MS")


def crc8_atm(data, crc=0x00):
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


fails = []


def chk(cond, msg):
    print(("[ok]   " if cond else "[FAIL] ") + msg)
    if not cond:
        fails.append(msg)


# ---------- CRC-8/ATM 正确性 ----------
chk(crc8_atm(b"123456789") == 0xF4, "CRC-8/ATM 标准向量 check: \"123456789\" -> 0xF4")

rec = bytes([0x05, 0x0E, 0x1F, 0x2A, 0x01, 0x34, 0x12, 0x64, 0x32] + [0x00] * (RECLEN - 1 - 9))
good = crc8_atm(rec[:RECLEN - 1])
torn = bytearray(rec[:RECLEN - 1]); torn[6] ^= 0x01
chk(crc8_atm(bytes(torn)) != good, "单 bit 撕裂被crc 检出")
torn2 = bytearray(rec[:RECLEN - 1]); torn2[1] = 0xFF
chk(crc8_atm(bytes(torn2)) != good, "时标整字节损坏被 crc 检出")
chk(CRC_OFF == RECLEN - 1, "crc 位于记录末字节 (off=%d, reclen=%d)" % (CRC_OFF, RECLEN))

# ---------- 布局 ----------
end = BASE + N * RECLEN - 1
print("[info] 日志区 0x%04X..0x%04X = %d B (%d 条 × %d B)"
      % (BASE, end, N * RECLEN, N, RECLEN))
chk(RECLEN % (PAGE // (PAGE // RECLEN)) == 0 and RECLEN <= PAGE and PAGE % RECLEN == 0,
    "记录长度整除页长 %d B（每页整放 %d 条）" % (PAGE, PAGE // RECLEN))
crosses = [i for i in range(N) if ((BASE + i * RECLEN) % PAGE) + RECLEN > PAGE]
chk(not crosses, "单条记录永不跨页 (违规槽: %s)" % crosses)

# head 两份必须是**不同字节单元**（endurance 按字节计）；同页但不同地址是允许的
chk(HEAD_A != HEAD_B, "head A(0x%04X) 与 head B(0x%04X) 是两个不同字节单元" % (HEAD_A, HEAD_B))
same_page_note = "同页(页%d)" % (HEAD_A // PAGE) if HEAD_A // PAGE == HEAD_B // PAGE else "跨页"
print("[info] head A/B %s —— endurance 按字节计，同页不同字节仍各自独立预算" % same_page_note)

# ---------- 磨损寿命核算 ----------
log_interval_s = LOG_PERIOD * SAMPLE_MS / 1000.0          # 一条日志的间隔
slot_interval_s = log_interval_s * N                      # 同一槽被覆盖的间隔
head_interval_s = log_interval_s * 2                      # A/B 轮换，每份的间隔

log_years = ENDURANCE * slot_interval_s / (365.25 * 24 * 3600)
head_years = ENDURANCE * head_interval_s / (365.25 * 24 * 3600)

print("")
print("[info] 落盘节拍: 每 %.0f s 一条 (= %d 次采样 × %d ms)"
      % (log_interval_s, LOG_PERIOD, SAMPLE_MS))
print("[info] RAM 黑匣子缓冲: %d 条 × %d ms = %.1f s 现场数据, %d B"
      % (RING_N, SAMPLE_MS, RING_N * SAMPLE_MS / 1000.0, N * 0 + RING_N * RECLEN))
print("")
print("[info] 磨损寿命 (endurance %d 次/字节):" % ENDURANCE)
print("       日志区  每槽 %8.0f s 覆盖一次 -> %7.2f 年" % (slot_interval_s, log_years))
print("       head A/B 每份 %6.0f s 写一次 -> %7.2f 年 (%.0f 天)"
      % (head_interval_s, head_years, head_years * 365.25))
print("       系统实际寿命 = 最短腿 = %.2f 年" % min(log_years, head_years))
print("")

chk(head_years >= 1.0, "head 寿命 ≥ 1 年 (实测 %.2f 年)；若回落至~23 天说明心跳被改回1s" % head_years)
chk(log_years >= 4.0, "日志区寿命 ≥ 4 年 (实测 %.2f 年)" % log_years)
# 黑匣子 RAM 开销
ring_bytes = RING_N * RECLEN
chk(ring_bytes <= 2048, "RAM 环形缓冲 %d B 在预算内 (≤2048B)" % ring_bytes)

print("")
if fails:
    print("[RESULT] 失败 %d 项" % len(fails))
    raise SystemExit(1)
print("[RESULT] 全部通过")
