#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_task.c 的 EEPROM 日志布局 / CRC / 磨损寿命自校验。

布局参数一律**从源码宏解析**，不在此硬编码，避免代码改动后校验脚本失配
（历史教训：曾把 12 B 记录的期望值写死，代码改成 16 B 后脚本一直报假 FAIL）。

两处设计约束的依据（注意二者**目的不同，不要混为一谈**）：
  · Write Endurance = 1,000,000 Write Cycles，**按字节(cell)计**。
    页(64B) 只是 I2C 写缓冲的传输粒度 => 就**寿命**而言，同页不同字节各享
    独立预算，跨页并不额外延寿。
  · datasheet 7.4 Write Cycle Timing：tWR 内部自定时写周期内
    "any attempts to read from or write to the memory array will not be
    processed"，正在被改写的单元处于不确定态 => 同页的 head 会被同一次
    高压写周期波及、一起变不确定。
    **head A/B 跨 64B 物理页的目的是「掉电物理故障隔离」，不是延寿。**
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
HEAD_RECLEN = macro("EEPROM_HEAD_RECLEN")
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

# head 两份必须**跨 64B 物理页**：tWR 写周期内同页数据一并处于不确定态，
# 同页则 A/B 双份会被同一次高压写周期一起打废，双份冗余失效。
head_a_page = HEAD_A // PAGE
head_b_page = HEAD_B // PAGE
chk(head_a_page != head_b_page,
    "head A(0x%04X, 页%d) 与 head B(0x%04X, 页%d) 分处不同 64B 物理页"
    % (HEAD_A, head_a_page, HEAD_B, head_b_page))

# 日志区必须与两个 head 所在页都不同页，且不与 head 区域地址重叠
log_page0 = BASE // PAGE
log_page1 = (BASE + N * RECLEN - 1) // PAGE
chk(log_page0 not in (head_a_page, head_b_page) and log_page1 not in (head_a_page, head_b_page),
    "日志区(页%d..页%d) 与 head A(页%d)/head B(页%d) 均不同页"
    % (log_page0, log_page1, head_a_page, head_b_page))

# head 各自占用的字节不得越出本页（避免踩到日志区）
chk(HEAD_A + HEAD_RECLEN <= (head_a_page + 1) * PAGE
    and HEAD_B + HEAD_RECLEN <= (head_b_page + 1) * PAGE,
    "head A/B 各5 B 均不越出本页边界")

# 区域地址不重叠
log_end = BASE + N * RECLEN
chk(HEAD_A + HEAD_RECLEN <= HEAD_B and HEAD_B + HEAD_RECLEN <= BASE,
    "地址不重叠: headA[0x%04X..) < headB[0x%04X..) <日志区(0x%04X..0x%04X)"
    % (HEAD_A, HEAD_B, BASE, log_end - 1))

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
