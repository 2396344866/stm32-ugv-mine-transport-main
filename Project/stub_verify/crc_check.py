# CRC-8/ATM 自校验：与 monitor_task.c 的 crc8_atm 同参数
def crc8_atm(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

# 标准测试向量 CRC-8/ATM(0x07,0x00): "123456789" -> 0xF4
v = crc8_atm(b"123456789")
print(f"[{'ok' if v == 0xF4 else 'FAIL'}] CRC-8/ATM 标准向量 check: got 0x{v:02X}, expect 0xF4")

# 撕裂检测：单 bit 翻转必被检出
rec = bytes([0x05, 0x0E, 0x1F, 0x2A, 0x01, 0x34, 0x12, 0x64, 0x32, 0x00, 0x00])
good = crc8_atm(rec)
torn = bytearray(rec); torn[6] ^= 0x01     # 翻转 raw 高字节 1 bit
print(f"[{'ok' if crc8_atm(bytes(torn)) != good else 'FAIL'}] 单 bit 撕裂被 crc 检出")
torn2 = bytearray(rec); torn2[1] = 0xFF    # 时标整字节损坏
print(f"[{'ok' if crc8_atm(bytes(torn2)) != good else 'FAIL'}] 时标字节损坏被 crc 检出")

# 布局核对
N, RECLEN, BASE = 128, 12, 0x0010
end = BASE + N*RECLEN - 1
print(f"[ok] 日志区 0x{BASE:04X}..0x{end:04X} = {N*RECLEN} B")
print(f"[{'ok' if (BASE % 64 == 0) else 'FAIL'}] 日志区按 64B 页对齐(每页 {64//RECLEN} 条)")
crosses = [i for i in range(N) if ((BASE + i*RECLEN) % 64) + RECLEN > 64]
print(f"[{'ok' if not crosses else 'FAIL'}] 单条记录不跨页 (违规槽: {crosses})")
print(f"[ok] head A@0x0000(5B) + head B@0x0008(5B), 落在页0内, 合计 {2*5} B")
print(f"[ok] 覆盖时长 {N} s ≈ {N//60}m{N%60:02d}s")
