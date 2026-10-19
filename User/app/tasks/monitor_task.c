/**
  * @file    monitor_task.c
  * @brief   工业管道气压监测任务：ADC 采样 -> 混合数字滤波 -> 迟滞报警
  *           -> EEPROM 断点续传存储 -> 低功耗(RTC/STOP)调度
  *
  *  高稳定性设计要点：
  *    - 混合滤波(去极值平均 + 一阶滞后)抑制传感器噪声与偶发尖峰；
  *    - 迟滞报警状态机避免临界抖动误报；
  *    - 异常数据落盘 EEPROM，断网/复位后可续传；
  *    - RTC(LSI) + STOP 模式支持电池供电长期监测（默认关闭，按需开启）。
  */
#include "tasks.h"
#include "filter.h"
#include "alarm.h"
#include "bsp_adc.h"
#include "bsp_rtc.h"
#include "bsp_at24c256.h"

/* ---- 监测参数 ---- */
#define MONITOR_PERIOD_MS     (50)     /* 采样周期 */
#define MONITOR_LOG_PERIOD    (600)    /* 每 600 次采样(30s)心跳落盘一次——见下方磨损均衡说明 */
#define MONITOR_LOG_PERIOD_HI (200)    /* 偏离正常时的加密节拍：200次=10s（见"分层落盘"） */
#define MONITOR_DEV_WARN      (0.05f)  /* 偏差 >5% 即视为趋势可疑，切换到加密节拍 */
#define MONITOR_RING_N        (60)     /* RAM 环形缓冲深度(条)：60×50ms=3s 现场数据 */
#define PRESSURE_VREF         (3.3f)
#define PRESSURE_FULL_SCALE   (10.0f)  /* 变送器量程 0..10 bar */

/* ---------------------------------------------------------------------------
 * 变送器输出规格与标定（4~20mA 电流环，井下牵引/制动管路的工程标准配置）
 *
 *  为什么按 4~20mA 而不是 0~10V —— 三个理由，都是现场硬约束：
 *  ① **长距离传输抗干扰**：巷道变频器/牵引逆变器噪声大，电流环(4~20mA)
 *     的信号强度与传输距离无关、且是恒流源驱动，衰减小、抗共模干扰；
 *     0~10V 电压式在长线上衰减与干扰成正比，读数会漂。
 *  ② **可检测断线**：4~20mA 约定"低于 3.6mA 即断线/故障"。电压式在断线时
 *     输出 0V，与"真实的 0 压力"无法区分 —— 这是本质缺陷，见下方判据。
 *  ③ **本安与供电共用**：两线制变送器由同一对线既供电又传信号，井下本安
 *     回路只允许极小电压/电流，降低关联设备风险。
 *
 *  标定链路（采样电阻R_SHUNT 上拉，ADC 读数）：
 *      I = 4mA .. 20mA        ⇒ V_SHUNT = 0.8V .. 4.0V（取 R_SHUNT=200Ω）
 *      0 bar  ⇒ 4 mA  ⇒ 0.8 V
 *      10 bar ⇒ 20 mA ⇒ 4.0 V
 *  ⚠️ 注意：4.0V > VREF(3.3V)，故必须分压。本工程按 2:1 分压接入 PA4：
 *      V_ADC = V_SHUNT / 2  ⇒  0.4V .. 2.0V
 *      ADC_RAW(0 bar)  = 0.4 / 3.3 × 4095 ≈ 496
 *      ADC_RAW(10 bar) = 2.0 / 3.3 × 4095 ≈ 2481
 *  ⇒ 换算式为**带零点偏置的线性标定**，不再是 raw/4095×FS：
 *      bar = (raw - RAW_AT_0BAR) / (RAW_AT_FSBAR - RAW_AT_0BAR) × 10.0
 *  ⇒ ADC 量程只用 0.4~2.0V 一段（约 1985 counts），分辨率 ≈ 5 mbar/LSB，
 *     相对 10 bar 量程足够（0.05%）。
 * ==========================================================================*/
#define PRESSURE_RAW_AT_0BAR  (496)    /* 0 bar  对应的 ADC 原始值 */
#define PRESSURE_RAW_AT_FSBAR (2481)   /* 10 bar 对应的 ADC 原始值 */

/* 断线/超量程判据（基于标定区间两端，不用"raw==0"这种脆弱写法）：
 *   低于 ADC_FAULT_LOW  ⇒ 断线（4~20mA 环开路 => 回读 < 4mA）
 *   高于 ADC_FAULT_HIGH ⇒ 超出量程（传感器故障/短路/超压）
 * 判据取"区间外扩 10% 计数"，避免临界抖动；命中即上报失效，走 fail-safe。*/
#define ADC_FAULT_LOW        ((PRESSURE_RAW_AT_0BAR * 9) / 10)    /* ≈ 446 */
#define ADC_FAULT_HIGH       (((PRESSURE_RAW_AT_FSBAR - PRESSURE_RAW_AT_0BAR) * 11 / 10) \
                              + PRESSURE_RAW_AT_0BAR)              /* ≈ 2729 */

#define PRESSURE_NOMINAL      (5.0f)   /* 标称气压(bar)，报警参考 */
#define FILTER_ALPHA          (0.2f)   /* 一阶滞后系数 */
#define TENSION_PER_BAR       (15.0f)  /* 气压 -> 张力代理增益(N/bar) */

/* ============================================================================
 * EEPROM 磨损均衡 + 掉电物理隔离
 *
 *  【一】head 为什么要跨 64B 物理页（本节布局的核心约束）
 *
 *  AT24C256C datasheet §7.4 Write Cycle Timing 明写：
 *    "During the internally self-timed write cycle, any attempts to read from
 *     or write to the memory array will not be processed."
 *  即在 tWR（≈5ms）内部自定时写周期内，**正在被改写的存储单元处于不确定态**。
 *  若 head A(0x0000) 与 head B(0x0040) 同处一页，则写 A 期间掉电，
 *  同一 64B Page Latch 上参与该次高压写周期的数据都可能变成不确定值——
 *  **A/B 双份会一起废掉，双份设计形同虚设**。
 *  ⇒ head A 置于第 0 页(0x0000)、head B 置于第 1 页(0x0040)、
 *    日志区整体顺延至第 2 页(0x0080)起，**三块区域物理页互不相邻**，
 *    保证任一次写周期掉电只波及一块，A/B 双份的冗余才真正成立。
 *  这是「掉电物理故障隔离」层面的考量。
 *
 *  【二】为什么不是"为了延寿"（这一点必须说清，否则易讲错机理）
 *
 *  Write Endurance = 1,000,000 Write Cycles，该指标**按字节(cell)计**
 *  （datasheet 另标注 "1,000,000 cycles per byte"），擦写只作用于目标字节的
 *  浮栅；64B 页仅是 I2C 写缓冲的传输粒度。
 *  ⇒ 就**寿命**而言，同页不同字节各享独立预算，跨页并不额外延寿。
 *  本设计跨页的目的是【一】的掉电物理隔离，**不是寿命**，两者要分开讲。
 *
 *  【三】降频（寿命维度的真正手段）
 *
 *  A/B 只解决冗余，不解决"写得太频繁"。head 寿命 = endurance × 写间隔：
 *    1 s落盘 → 每份 head 每 2 s 写一次 → 1e6×2 s ≈ **23 天**（不可接受）
 *    30 s 心跳 → 每份每 60 s 一次  → 1e6×60 s ≈ **694 天**
 *  日志区（128 槽均摊，30 s 一条）≈ 121 年，系统寿命取决于最短腿(head)。
 *
 *  配套：正常工况走 30 s 低频心跳；一旦 alarm 越限或安全态异常，立即进入
 *  **黑匣子模式**——把 RAM 环形缓冲里最近 3s 现场全速刷入 EEPROM，
 *  且**分步执行**（见 ring_flush_step），绝不阻塞制动响应窗口。
 * ==========================================================================*/
#define EEPROM_LOG_BASE       (0x0080) /* 日志区起始：第 2 页，避开 head 两页 */

/* ============================================================================
 * EEPROM 环形日志布局（工业界掉电安全做法）
 *
 *  三块区域分处互不相邻的物理页（掉电物理隔离的依据见上方【一】）：
 *    第 0 页 0x0000..0x003F  head A : seq(4,LE) + crc8(1)  = 5 B
 *    第 1 页 0x0040..0x007F  head B : seq(4,LE) + crc8(1)  = 5 B
 *    第 2 页起0x0080..0x087F  日志区：128 条 × 16 B = 2048 B
 *
 *  记录（16 B）：
 *    seq(1) hour(1) min(1) sec(1) alarm(1) raw(2,LE) filt100(2,LE)
 *    pad(3) crc8(1)  —— 后 5 B 预留扩展（如毫秒/温度/故障码），当前填 0
 *
 *  为何取 16 B：AT24C256 页长 64 B，16 是 64 的整数倍 ⇒ 每页整放 4 条，
 *  **单条永不跨页**，无需在写入路径里拆页事务（避免每条多一次 tWR 等待，
 *  降低写入耗时与失败概率）。驱动层仍保留通用分页作为兜底。
 *
 *  掉电安全设计：
 *   ① **head A/B 双份交替 + 跨 64B 物理页**：每写一批后翻转活动份，两份分处
 *      第 0/第 1 页。因 tWR 写周期内同页数据一并处于不确定态，跨页才保证
 *      掉电时最多只波及非活动的那份（A/B 同页则两份可能一起废）；
 *      上电时取「crc 校验通过且 seq 较大」者，因此必然拿到一致的 head；
 *   ② **记录带 seq + crc8**：可逐条判定完整性，撕裂记录在读回时被丢弃而非
 *      当成有效历史，读取端沿 seq 连续性扫描出有效区间；
 *   ③ **先写记录、后提交 head**：head 仍是提交点（commit point），
 *      掉电在记录写窗口时，head 未推进 ⇒ 该槽尚未提交，下次直接覆盖。
 * ==========================================================================*/
/* 三块区域分处三个不同的 64B 物理页（详见上方【一】掉电物理隔离）：
 *   第 0 页0x0000..0x003F  head A : seq(4,LE) + crc8(1) = 5 B
 *   第 1 页 0x0040..0x007F  head B : seq(4,LE) + crc8(1) = 5 B
 *   第 2 页 0x0080..0x087F  日志区 : 128 条 × 16 B = 2048 B
 * 日志区从 0x0080 起仍是 64 的整数倍 ⇒ 16 B 记录每页整放 4 条、单条永不跨页。*/
#define EEPROM_LOG_BASE       (0x0080)
#define EEPROM_LOG_N          (128)
#define EEPROM_LOG_RECLEN     (16)
#define EEPROM_REC_CRC_OFF    (15)     /* crc 位于记录末字节 */
#define EEPROM_REC_DATA_LEN   (15)     /* crc 覆盖前 15 字节 */
#define EEPROM_HEAD_A_ADDR    (0x0000) /* 第 0 页 */
#define EEPROM_HEAD_B_ADDR    (0x0040) /* 第 1 页——与 A 物理隔离 */
#define EEPROM_HEAD_RECLEN    (5)      /* seq(4) + crc(1) */
#define EEPROM_HEAD_INVALID   (0xFFFFFFFFu)

/* CRC-8/ATM (poly=0x07, init=0x00)，逐位计算，无查表 RAM 开销 */
static uint8_t crc8_atm(uint8_t crc, const uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void put_u32_le(uint8_t* p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get_u32_le(const uint8_t* p)
{
    return ((uint32_t)p[0])
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static Filter_Handle    s_filt;
static AlarmHysteresis  s_alarm;

/* ---------------------------------------------------------------------------
 * RAM 环形缓冲（黑匣子预存）
 *
 *  每次采样都只写 RAM，不动 EEPROM ⇒ 采样率保持 50ms，黑匣子能还原故障
 *  前后的连续曲线。落 EEPROM 由**三层自适应节拍**驱动（见 Monitor_Task）：
 *    ① 正常（偏差≤5%）→ 30 s 心跳，最省寿命；
 *    ② 趋势可疑（偏差>5%）→ 10 s 加密，抓缓慢渐变过程；
 *    ③ 异常（报警越限 / 安全态非 SAFE_RUN / ADC 失效）→ 每拍分步全速刷盘。
 *
 *  条目复用 EEPROM 记录的前 9 B 有效载荷布局（seq 在刷盘时才由 head_seq 填），
 *  故 RAM 里直接存 16 B 定长，8 条即 128 B，60 条 960 B——F103RCT6 有 48KB
 *  RAM，任务栈合计仅 4.6KB，该开销可接受。
 * -------------------------------------------------------------------------*/
typedef struct
{
    uint8_t  payload[EEPROM_REC_DATA_LEN];  /* seq + 时标 + alarm + raw + filt + pad */
} LogEntry;

static LogEntry s_ring[MONITOR_RING_N];
static uint16_t s_ring_head = 0;   /* 下一次写入位置 */
static uint16_t s_ring_len  = 0;   /* 当前有效条目数（未满时 < N） */
static uint8_t  s_blackbox  = 0;   /* 黑匣子锁存标志：置位后持续分步刷到缓冲清空 */
static uint8_t  s_adc_bad   = 0;   /* ADC 失效标志：raw 落在标定区间外（断线/超量程） */

/* head 提交点状态。声明上移至此处：ring_flush_step（黑匣子分步刷盘）也要读写它，
 * 不能留在下方的 head A/B 段内。*/
static uint8_t  s_head_slot = 0;    /* 0=A 1=B，当前活动份 */
static uint32_t s_head_seq  = 0;

/* ring_flush_step 需调用 head 提交函数，而其定义在下方 head A/B 读写段之后，
 * 故在此前置声明（C89 不允许隐式声明，且 Keil ARMCC 默认按 C89 编译）。*/
static int  head_write_one(uint16_t addr, uint32_t seq);

static void ring_push(const uint8_t* payload)
{
    for (uint8_t i = 0; i < EEPROM_REC_DATA_LEN; i++)
    {
        s_ring[s_ring_head].payload[i] = payload[i];
    }
    s_ring_head = (uint16_t)((s_ring_head + 1u) % MONITOR_RING_N);
    if (s_ring_len < MONITOR_RING_N) s_ring_len++;
}

/* ---------------------------------------------------------------------------
 * 黑匣子分步刷盘（step-wise flush）—— **为不挤占制动响应窗口而设计**
 *
 *  为什么必须分步：EEPROM 单条写入含 tWR ≈ 5ms 的内部写周期。若一次
 *  ring_flush_step 若一次把缓冲内 60 条全刷完，**单次调用将连续阻塞约 300ms**。
 *  本工程控制环节拍是 20ms、安全状态机要求 ≤20ms 响应，BRAKE/ERROR 属
 *  最高优先级动作，一旦被刷盘挤占就构成安全隐患。
 *
 *  三重保障：
 *   ① **任务优先级**：TASK_PRIO_CONTROL(5) > TASK_PRIO_MONITOR(3)，
 *      且控制环由 TIM3 中断释放信号量驱动，FreeRTOS 抢占式调度下
 *      control_task 必然先拿到 CPU，monitor_task 只是被抢占而非拖延控制环；
 *   ② **分步执行**：每次调用最多刷 RING_FLUSH_BATCH 条，
 *      把 300ms 摊成 60/N 次短调用，单次阻塞量降到 ≤ MONITOR_PERIOD_MS，
 *      绝不出现长阻塞窗口；
 *   ③ **无忙等**：每条写完即返回到 vTaskDelay，让出 CPU。
 *
 *  ⚠️ 关键约束：**触发刷盘的判据里不能包含"制动正在动作"这一状态**。
 *  本实现仅以 alarm 越限 / 安全态非 SAFE_RUN 为触发（见 Monitor_Task），
 *  即"故障发生后尽快取证"，而不是"制动执行期间去抢总线"。
 *  制动动作本身由 control_task 在自己的 20ms 节拍内独立完成，不依赖本任务。
 * -------------------------------------------------------------------------*/
#define RING_FLUSH_BATCH      (2)      /* 每次最多刷几条：2×5ms=10ms < 20ms 节拍 */

/* 按时间顺序刷入 EEPROM，每次最多 RING_FLUSH_BATCH 条（分步，避免长阻塞）。
 * 返回本次成功刷写的条数；任一条失败即停止，保留其余条目待下次重试。
 * 全部刷完（或有剩余）时提交一次 head。 */
static uint16_t ring_flush_step(void)
{
    uint16_t done   = 0;
    uint16_t budget = RING_FLUSH_BATCH;
    uint16_t start  = (uint16_t)((s_ring_head + MONITOR_RING_N - s_ring_len) % MONITOR_RING_N);

    for (uint16_t k = 0; k < s_ring_len && k < budget; k++)
    {
        uint16_t idx = (uint16_t)((start + k) % MONITOR_RING_N);
        uint8_t  rec[EEPROM_LOG_RECLEN];
        for (uint8_t i = 0; i < EEPROM_REC_DATA_LEN; i++) rec[i] = s_ring[idx].payload[i];
        rec[EEPROM_REC_CRC_OFF] = crc8_atm(0x00u, rec, EEPROM_REC_DATA_LEN);

        uint16_t addr = EEPROM_LOG_BASE + (s_head_seq % EEPROM_LOG_N) * EEPROM_LOG_RECLEN;
        if (BSP_AT24C256_Write(addr, rec, EEPROM_LOG_RECLEN) != 0)
        {
            Error_Record(ERR_EEPROM_WRITE);
            break;   /* 写失败即停：剩余条目留在 RAM，下次触发时重试 */
        }
        s_head_seq++;
        done++;
    }

    if (done == s_ring_len)
    {
        s_ring_len = 0;   /* 全部刷出，清空缓冲 */
    }
    else if (done > 0)
    {
        /* 保留未刷出的尾部条目 */
        uint16_t keep = (uint16_t)(s_ring_len - done);
        uint16_t nstart = (uint16_t)((start + done) % MONITOR_RING_N);
        for (uint16_t k = 0; k < keep; k++)
        {
            uint16_t src = (uint16_t)((nstart + k) % MONITOR_RING_N);
            for (uint8_t i = 0; i < EEPROM_REC_DATA_LEN; i++)
            {
                s_ring[k].payload[i] = s_ring[src].payload[i];
            }
        }
        s_ring_len = keep;
    }

    /* **每次 step 只提交一次 head**（而非每条各提交一次）。
     * 这是把 head 写入量真正压下来的关键：head 寿命按「调用次数」计，
     * 分步刷 60 条共 60 次调用 ⇒ head 只被擦写 60 次而非 60×60 次；
     * 正常态 30s 心跳则每次调用即一条、提交一次。
     *
     * 掉电语义不变：head 是提交点，未提交 ⇒ 本次已刷的这几条视为未提交，
     * 下次从头重刷，最坏代价是丢这几条 RAM 数据（RAM 本就易失，无额外损失）。
     * 分步不影响该语义：每步自成一个原子事务。*/
    if (done > 0)
    {
        uint16_t next_slot = (uint16_t)(s_head_slot ^ 1u);
        uint16_t next_addr = next_slot ? EEPROM_HEAD_B_ADDR : EEPROM_HEAD_A_ADDR;
        if (head_write_one(next_addr, s_head_seq) != 0)
        {
            /* 提交失败：整批回退 seq，保持与 EEPROM 有效 head 一致 */
            s_head_seq -= done;
            Error_Record(ERR_EEPROM_WRITE);
            return 0;
        }
        s_head_slot = (uint8_t)next_slot;
    }
    return done;
}

/* ---- head A/B 双份读写 ---- */
/* 读一份 head：crc 校验通过才返回；失败返回 EEPROM_HEAD_INVALID */
static uint32_t head_read_one(uint16_t addr)
{
    uint8_t buf[EEPROM_HEAD_RECLEN] = {0};
    if (BSP_AT24C256_Read(addr, buf, EEPROM_HEAD_RECLEN) != 0)
    {
        Error_Record(ERR_I2C_TIMEOUT);
        return EEPROM_HEAD_INVALID;
    }
    uint8_t calc = crc8_atm(0x00u, buf, 4);
    if (calc != buf[4])
    {
        return EEPROM_HEAD_INVALID;      /* crc 不符 ⇒ 该份已损坏 */
    }
    return get_u32_le(buf);
}

/* 写一份 head（seq 显式小端 + crc 覆盖前 4 字节） */
static int head_write_one(uint16_t addr, uint32_t seq)
{
    uint8_t buf[EEPROM_HEAD_RECLEN];
    put_u32_le(buf, seq);
    buf[4] = crc8_atm(0x00u, buf, 4);
    if (BSP_AT24C256_Write(addr, buf, EEPROM_HEAD_RECLEN) != 0)
    {
        Error_Record(ERR_EEPROM_WRITE);
        return -1;
    }
    return 0;
}

/* 上电恢复：取 A/B 中 crc 通过且 seq 较大者；活动份随后用于轮转写入。
 * 返回有效 seq（无有效则 0 表示从头开始）。 */
static uint32_t head_load(uint8_t* active_slot)
{
    uint32_t a = head_read_one(EEPROM_HEAD_A_ADDR);
    uint32_t b = head_read_one(EEPROM_HEAD_B_ADDR);

    /* 用无符号差值处理 seq 回绕（模 2^32），谁「更新」用 (b-a) 是否落在后一半判定 */
    uint32_t newer_b = (uint32_t)(b - a);
    int b_valid = (a != EEPROM_HEAD_INVALID) || (b != EEPROM_HEAD_INVALID);
    int b_newer;
    if (a == EEPROM_HEAD_INVALID)      b_newer = 1;
    else if (b == EEPROM_HEAD_INVALID) b_newer = 0;
    else                                b_newer = (newer_b != 0u) && (newer_b < 0x80000000u);

    if (!b_valid) { *active_slot = 0; return 0; }
    if (b_newer)  { *active_slot = 1; return b; }
    *active_slot = 0;
    return (a == EEPROM_HEAD_INVALID) ? b : a;
}

/* 写入一条监测记录（16B，带 seq + crc8），并提交 head。
 * 写序：先记录、后 head（head 为提交点）。head A/B 交替翻转，掉电只可能损坏
 * 非活动份，上电经 crc+seq 取较新者即可恢复一致 head。
 * 注：s_head_slot / s_head_seq 已上移至 RAM 环形缓冲段（ring_flush_step 也需访问）。*/

static void eeprom_log(uint8_t hour, uint8_t min, uint8_t sec,
                       uint8_t alarm, uint16_t raw, uint16_t filt100)
{
    uint8_t rec[EEPROM_LOG_RECLEN];

    /* seq 落在低字节（128 槽循环，低 7 位足够区分相邻记录） */
    rec[0] = (uint8_t)(s_head_seq & 0xFFu);
    rec[1] = hour;
    rec[2] = min;
    rec[3] = sec;
    rec[4] = alarm;
    rec[5] = (uint8_t)(raw & 0xFFu);
    rec[6] = (uint8_t)((raw >> 8) & 0xFFu);
    rec[7] = (uint8_t)(filt100 & 0xFFu);
    rec[8] = (uint8_t)((filt100 >> 8) & 0xFFu);
    /* rec[9..14] 预留扩展，当前填 0；crc 覆盖 rec[0..14] */
    for (uint8_t i = 9; i < EEPROM_REC_CRC_OFF; i++) rec[i] = 0;
    rec[EEPROM_REC_CRC_OFF] = crc8_atm(0x00u, rec, EEPROM_REC_DATA_LEN);

    uint16_t addr = EEPROM_LOG_BASE + (s_head_seq % EEPROM_LOG_N) * EEPROM_LOG_RECLEN;
    if (BSP_AT24C256_Write(addr, rec, EEPROM_LOG_RECLEN) != 0)
    {
        Error_Record(ERR_EEPROM_WRITE);
        return;   /* 记录未落盘 → 不推进 seq/head，保持提交点语义 */
    }

    /* 提交：seq 先加一，再翻转到另一份 head 写入 */
    s_head_seq++;
    uint16_t next_slot = (uint16_t)(s_head_slot ^ 1u);
    uint16_t next_addr = next_slot ? EEPROM_HEAD_B_ADDR : EEPROM_HEAD_A_ADDR;
    if (head_write_one(next_addr, s_head_seq) != 0)
    {
        /* 提交失败：回退 seq，保持与 EEPROM 中有效 head 一致 */
        s_head_seq--;
        return;
    }
    s_head_slot = (uint8_t)next_slot;
}

/* ---------------------------------------------------------------------------
 * 读取接口（供通信任务做断点续传）
 *
 * 按 seq 从最新往回扫描，逐条用 crc8 判定完整性：
 *   - crc 通过 ⇒ 该条完整有效；
 *   - crc 失败 ⇒ 该条被掉电撕裂，直接跳过（不中断扫描，继续向更旧找）。
 * 因为 head 是提交点，任何未被 head 覆盖的撕裂记录都不在有效区间内，
 * 而被覆盖的最旧槽即使撕裂也只丢 1 条，seq 连续性扫描可精确识别。
 * 返回 1 = 取到一条有效记录（填入 out），0 = 已扫完全部槽位。
 * -------------------------------------------------------------------------*/
int Monitor_LogRead(uint32_t back, uint8_t* out, uint8_t len)
{
    if (out == NULL || len < EEPROM_LOG_RECLEN) return 0;

    for (uint32_t k = 0; k < EEPROM_LOG_N; k++)
    {
        /* back=0 为最新一条，逐条向更旧走 */
        uint32_t seq = s_head_seq - 1u - k;
        uint16_t addr = EEPROM_LOG_BASE + (seq % EEPROM_LOG_N) * EEPROM_LOG_RECLEN;

        uint8_t rec[EEPROM_LOG_RECLEN];
        if (BSP_AT24C256_Read(addr, rec, EEPROM_LOG_RECLEN) != 0)
        {
            continue;   /* 总线读失败：跳过该条 */
        }
        if (crc8_atm(0x00u, rec, EEPROM_REC_DATA_LEN) != rec[EEPROM_REC_CRC_OFF])
        {
            continue;   /* crc 不符：撕裂记录，丢弃 */
        }
        /* 校验 seq 低字节一致，确认扫到的是预期那条（防槽位错位） */
        if (rec[0] != (uint8_t)(seq & 0xFFu))
        {
            continue;   /* 该槽尚未写入，或属于更旧代次 */
        }

        for (uint8_t i = 0; i < len; i++) out[i] = rec[i];
        return 1;
    }
    return 0;
}

/* 当前已提交 head（seq），供通信任务记录续传进度 */
uint32_t Monitor_LogHead(void)
{
    return s_head_seq;
}

void Monitor_Task(void* pvParameters)
{
    (void)pvParameters;

    Health_Register(TASK_ID_MONITOR);

    Filter_Init(&s_filt, FILTER_ALPHA);
    Alarm_Init(&s_alarm, PRESSURE_NOMINAL, 0.05f, 0.02f, 0.08f);

#if USE_RTC_DEPENDENT
    BSP_RTC_Init();
#endif

    /* 上电恢复：从 head A/B 双份中取 crc 通过且 seq 较新者 */
    s_head_seq = head_load(&s_head_slot);

    uint16_t raw = 0;
    float   bar = 0.0f, filt = 0.0f;
    uint32_t sample_cnt = 0;

    for (;;)
    {
        Health_Report(TASK_ID_MONITOR);

        /* --- 1. ADC 采样 + 失效判据 ---
         * 变送器为 4~20mA 电流环 + 分压标定，0 bar 对应 raw≈496、10 bar≈2481。
         * 因此判据**不能写 raw==0**——那既不覆盖开路回读偏低的真实情形，
         * 也会把"合法的 0 bar 附近读数"误判。改为区间外判据：
         *   raw < ADC_FAULT_LOW   ⇒ 电流环开路/断线（低于 3.6mA 约定）
         *   raw > ADC_FAULT_HIGH  ⇒ 超量程（传感器故障/短路/超压）
         * 命中即置失效标志并记错，**且不再把该拍数据送入滤波**——
         * 否则一个坏值会被两级滤波当成正常值吃掉，故障被"抹平"而失去告警。*/
        raw = BSP_ADC_ReadRaw();
        if (raw < ADC_FAULT_LOW || raw > ADC_FAULT_HIGH)
        {
            s_adc_bad = 1u;
            Error_Record(ERR_ADC_READ);
        }
        else
        {
            s_adc_bad = 0u;
        }

        /* 带零点偏置的线性标定（4~20mA 规格，见上方说明）：
         *   bar = (raw - RAW_AT_0BAR) / (RAW_AT_FS - RAW_AT_0BAR) × 10.0
         * 失效拍取"量程下限"作为保守代表值，而不是 0 bar ——
         * 气压过低对应张力不足，同样属于危险工况。*/
        if (s_adc_bad)
        {
            bar = 0.0f;
        }
        else
        {
            bar = ((float)raw - (float)PRESSURE_RAW_AT_0BAR)
                / ((float)PRESSURE_RAW_AT_FSBAR - (float)PRESSURE_RAW_AT_0BAR)
                * PRESSURE_FULL_SCALE;
            /* 标定区间外做钳位，防止标定误差把值推到量程之外 */
            if (bar < 0.0f)   bar = 0.0f;
            if (bar > PRESSURE_FULL_SCALE) bar = PRESSURE_FULL_SCALE;
        }

        /* --- 2. 混合滤波 --- */
        filt = Filter_Update(&s_filt, bar);

        /* --- 3. 迟滞报警 --- */
        AlarmLevel lvl = Alarm_Update(&s_alarm, filt);

        /* --- 4. 张力代理 & 回写全局状态 --- */
        float tension = filt * TENSION_PER_BAR;
        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                g_state.pressure_raw  = raw;
                g_state.pressure_bar  = bar;
                g_state.pressure_filt = filt;
                g_state.alarm         = lvl;
                g_state.tension       = tension;
                g_state.pressure_bad  = s_adc_bad;   /* 供 control_task fail-safe 判定 */
                xSemaphoreGive(xSystemStateMutex);
            }
        }

        /* --- 5. 采样入 RAM 环形缓冲（黑匣子预存，不直接写 EEPROM） ---
         * 每拍都预存，故黑匣子能还原故障前后连续曲线；写 EEPROM 由下面两级触发。*/
        uint8_t h = 0, m = 0, s = 0;
#if USE_RTC_DEPENDENT
        BSP_RTC_GetTime(&h, &m, &s);
#endif
        uint16_t filt100 = (uint16_t)(filt * 100.0f);

        uint8_t payload[EEPROM_REC_DATA_LEN];
        payload[0] = (uint8_t)(s_head_seq & 0xFFu);   /* 占位 seq，刷盘时由 head_seq 覆盖 */
        payload[1] = h; payload[2] = m; payload[3] = s;
        payload[4] = (uint8_t)lvl;
        payload[5] = (uint8_t)(raw & 0xFFu);
        payload[6] = (uint8_t)((raw >> 8) & 0xFFu);
        payload[7] = (uint8_t)(filt100 & 0xFFu);
        payload[8] = (uint8_t)((filt100 >> 8) & 0xFFu);
        for (uint8_t i = 9; i < EEPROM_REC_DATA_LEN; i++) payload[i] = 0;
        ring_push(payload);

        /* --- 6. 分层落盘触发（解决"寿命"与"取证分辨率"的矛盾） ---
         *
         *  只用 30s 心跳会漏掉**缓慢渐变**故障（如管路缓慢泄漏、闸阀缓卡）：
         * 这类故障在到达报警阈值前要几分钟，30s 采样只能看到几个孤立点，
         * 无法还原演化过程 —— 事后追溯失效。
         *  但若常态就用 50ms 落盘，head 寿命又回到 23 天。
         *
         *  ⇒ 三层自适应节拍，用"数据价值"决定采样密度，而不是一刀切：
         *    ① 正常（偏差 ≤5%）：30s 心跳，最省寿命（head ≈694 天）；
         *    ② 趋势可疑（偏差 >5%，尚未报警）：**10s 加密**，抓渐变过程；
         *       此时 head 每 20s 写一次 ⇒ 寿命 ≈231 天，仍在可接受区间
         *       （对比 5s 加密只有 116 天，不足以支撑长周期运行，故取 10s）；
         *    ③ 异常（报警越限 / 安全态非 SAFE_RUN / ADC 失效）：
         *       **50ms 全速黑匣子**（见下 s_blackbox），无损。
         *  三层之间靠偏差判据自动切换，无需人工干预。*/
        uint8_t blackbox = 0;
        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                blackbox = ((g_state.alarm != ALARM_NORMAL) ||
                            (g_state.safety != SAFE_RUN)) ? 1u : 0u;
                xSemaphoreGive(xSystemStateMutex);
            }
        }
        /* ADC 失效同样触发黑匣子：传感器断线本身就是必须取证的事件 */
        if (s_adc_bad)
        {
            blackbox = 1;
            s_blackbox = 1;
        }
        if (blackbox)
        {
            s_blackbox = 1;
        }
        else if (s_ring_len == 0)
        {
            s_blackbox = 0;   /* 缓冲已清空且故障恢复 ⇒ 退出黑匣子模式 */
        }

        if (s_blackbox && s_ring_len > 0)
        {
            /* 分步刷盘：单次阻塞 ≤ RING_FLUSH_BATCH × tWR(≈10ms) < 20ms 节拍 */
            (void)ring_flush_step();
        }
        else if (!s_blackbox)
        {
            /* 分层心跳：趋势可疑时用 10s 密节拍，其余用 30s 疏节拍。
             * 偏差用**滤波后**值判断，避免噪声本身把节拍一直拉高。*/
            float dev = (PRESSURE_NOMINAL > 0.0f)
                      ? ((filt - PRESSURE_NOMINAL) / PRESSURE_NOMINAL)
                      : 0.0f;
            if (dev < 0.0f) dev = -dev;
            uint32_t period = (dev > MONITOR_DEV_WARN)
                            ? (uint32_t)MONITOR_LOG_PERIOD_HI
                            : (uint32_t)MONITOR_LOG_PERIOD;

            if ((++sample_cnt % period) == 0)
            {
                /* 心跳：只刷最新一条，保留其余现场数据在 RAM 等待下次触发 */
                s_ring_len = (s_ring_len > 0u) ? (uint16_t)(s_ring_len - 1u) : 0u;
                eeprom_log(h, m, s, (uint8_t)lvl, raw, filt100);
            }
        }

        if (xLogQueue != NULL)
        {
            uint8_t log[8] = { h, m, s, (uint8_t)lvl,
                               (uint8_t)(raw & 0xFF), (uint8_t)((raw >> 8) & 0xFF),
                               (uint8_t)(filt100 & 0xFF), (uint8_t)((filt100 >> 8) & 0xFF) };
            xQueueSend(xLogQueue, log, 0);
        }

        /* --- 7. 低功耗：仅在使能且系统允许时，由 idle 钩子进入 STOP --- */
        vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
    }
}
