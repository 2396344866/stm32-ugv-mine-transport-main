/**
 * @file    test_pressure_scale.c
 * @brief   气压采样链路的标定 / 失效判据 / 分层落盘节拍 单测（PC 端运行）
 *
 * 本轮修复的两个真实缺陷，各配断言：
 *   【缺陷1】原换算式 `raw/4095.0f * VREF/VREF * FS` —— VREF 自我相消，
 *           等价于假设"变送器输出满量程恰好等于3.3V"，且**没有零点偏置**。
 *           对 4~20mA(0bar→raw≈496) 的规格会整体错位，0 bar 也算不对。
 *   【缺陷2】断线判据写成 `raw == 0` —— 只记Error 不报警，
 *           fail-safe 方向有洞（断线后 tension=0 被当成"安全"放行）。
 *
 * 覆盖：
 *   ① 标定端点与单调性：0bar→0.0，10bar→10.0，中间单调；
 *   ② 分辨率：1 LSB 的气压当量在工程可接受范围（≤0.01 bar）；
 *   ③ **失效判据**：区间内不误报、低于下限（断线）与高于上限（超量程）必报；
 *   ④ **失效拍不进滤波**：失效时代表值取量程下限而非 0，避免"读不到=正常"；
 *   ⑤ 分层节拍：正常 30s、趋势可疑 5s、异常走黑匣子，head 寿命均达标。
 */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdarg.h>

/* ===== SKELETON: 与 monitor_task.c 保持一致 ===== */
#define PRESSURE_FULL_SCALE     (10.0f)
#define PRESSURE_NOMINAL        (5.0f)
#define PRESSURE_RAW_AT_0BAR    (496)
#define PRESSURE_RAW_AT_FSBAR   (2481)
#define ADC_FAULT_LOW           ((PRESSURE_RAW_AT_0BAR * 9) / 10)
#define ADC_FAULT_HIGH          (((PRESSURE_RAW_AT_FSBAR - PRESSURE_RAW_AT_0BAR) * 11 / 10) \
                                  + PRESSURE_RAW_AT_0BAR)
#define MONITOR_DEV_WARN        (0.05f)
#define MONITOR_LOG_PERIOD      (600)
#define MONITOR_LOG_PERIOD_HI   (200)

static uint8_t adc_bad_of(uint16_t raw)
{
    return (uint8_t)((raw < ADC_FAULT_LOW || raw > ADC_FAULT_HIGH) ? 1u : 0u);
}

static float bar_of(uint16_t raw)
{
    if (adc_bad_of(raw)) return 0.0f;              /* 失效拍不进标定 */
    float b = ((float)raw - (float)PRESSURE_RAW_AT_0BAR)
            / ((float)PRESSURE_RAW_AT_FSBAR - (float)PRESSURE_RAW_AT_0BAR)
            * PRESSURE_FULL_SCALE;
    if (b < 0.0f) b = 0.0f;
    if (b > PRESSURE_FULL_SCALE) b = PRESSURE_FULL_SCALE;
    return b;
}

/* 失效拍的代表值：取量程下限（低压=张力不足=危险），而非 0=正常 */
static float bar_repr(uint16_t raw)
{
    return adc_bad_of(raw) ? 0.0f : bar_of(raw);
}

static int period_of(float filt)
{
    float dev = ((filt - PRESSURE_NOMINAL) / PRESSURE_NOMINAL);
    if (dev < 0.0f) dev = -dev;
    return (dev > MONITOR_DEV_WARN) ? MONITOR_LOG_PERIOD_HI : MONITOR_LOG_PERIOD;
}
/* ===== SKELETON 结束 ===== */

static int s_pass = 0, s_fail = 0;
static void check(int cond, const char* fmt, ...)
{
    char msg[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    if (cond) { printf("  [ok]   %s\n", msg); s_pass++; }
    else      { printf("  [FAIL] %s\n", msg); s_fail++; }
}
static int near(float a, float b, float eps)
{
    return fabsf(a - b) <= eps;
}

int main(void)
{
    printf("[气压标定 · 失效判据 · 分层节拍]\n");
    printf("[info] 标定: %d raw=0bar .. %d raw=10bar; 失效区间 [%d, %d]\n\n",
           PRESSURE_RAW_AT_0BAR, PRESSURE_RAW_AT_FSBAR, ADC_FAULT_LOW, ADC_FAULT_HIGH);

    /* ---------- ① 标定端点 ---------- */
    check(near(bar_of(PRESSURE_RAW_AT_0BAR), 0.0f, 0.01f),
          "标定下端点 raw=%d -> 0.00 bar", PRESSURE_RAW_AT_0BAR);
    check(near(bar_of(PRESSURE_RAW_AT_FSBAR), PRESSURE_FULL_SCALE, 0.01f),
          "标定上端点 raw=%d -> 10.00 bar", PRESSURE_RAW_AT_FSBAR);
    check(near(bar_of((PRESSURE_RAW_AT_0BAR + PRESSURE_RAW_AT_FSBAR) / 2),
               PRESSURE_FULL_SCALE / 2, 0.02f),
          "标定中点 -> 5.00 bar（线性正确）");

    /* ---------- ② 单调性 + 分辨率 ---------- */
    int mono = 1;
    float prev = -1.0f;
    for (int raw = ADC_FAULT_LOW + 1; raw <= ADC_FAULT_HIGH - 1; raw++)
    {
        float b = bar_of((uint16_t)raw);
        if (b < prev - 1e-6f) { mono = 0; break; }
        prev = b;
    }
    check(mono, "整个有效区间内bar 随raw 单调递增");

    float lsb_bar = PRESSURE_FULL_SCALE / (float)(PRESSURE_RAW_AT_FSBAR - PRESSURE_RAW_AT_0BAR);
    check(lsb_bar <= 0.01f, "1 LSB 当量 %.4f bar <= 0.01 bar（分辨率足够）", lsb_bar);
    printf("[info] ADC 有效区间 %d counts, 分辨率 %.2f mbar/LSB\n",
           ADC_FAULT_HIGH - ADC_FAULT_LOW, lsb_bar * 1000.0f);

    /* ---------- ③ 失效判据 ---------- */
    check(adc_bad_of(0) == 1, "raw=0 判为失效（不再静默当成 0 bar）");
    check(adc_bad_of(ADC_FAULT_LOW - 1) == 1, "raw<%d 判为断线失效", ADC_FAULT_LOW);
    check(adc_bad_of(ADC_FAULT_HIGH + 1) == 1, "raw>%d 判为超量程失效", ADC_FAULT_HIGH);
    check(adc_bad_of(ADC_FAULT_LOW) == 0, "raw=%d(下边界) 不误报", ADC_FAULT_LOW);
    check(adc_bad_of(ADC_FAULT_HIGH) == 0, "raw=%d(上边界) 不误报", ADC_FAULT_HIGH);
    check(adc_bad_of(PRESSURE_RAW_AT_0BAR) == 0, "合法 0 bar 读数不误报");
    check(adc_bad_of(PRESSURE_RAW_AT_FSBAR) == 0, "合法 10 bar 读数不误报");

    /* ---------- ④ 失效拍不进滤波 ---------- */
    check(near(bar_repr(0), 0.0f, 1e-6f),
          "断线拍代表值 = 0 bar（气压过低本身即危险，不是'正常'）");
    check(bar_repr(0) == bar_repr(ADC_FAULT_LOW - 1),
          "断线区间内不同 raw 得到同一代表值（不被噪声扰动）");

    /* ---------- ⑤ 分层节拍 + head 寿命 ---------- */
    check(period_of(5.00f) == MONITOR_LOG_PERIOD, "正常(5.00bar) -> 30s 疏节拍");
    check(period_of(5.10f) == MONITOR_LOG_PERIOD, "偏高 2%% (5.10) -> 仍 30s");
    check(period_of(5.30f) == MONITOR_LOG_PERIOD_HI, "偏高 6%%(5.30) -> 10s 密节拍");
    check(period_of(4.70f) == MONITOR_LOG_PERIOD_HI, "偏低 6%%(4.70) -> 10s 密节拍");
    check(period_of(4.90f) == MONITOR_LOG_PERIOD, "偏低 2%%(4.90) -> 仍 30s");

    double years_lo = 1e6 * (MONITOR_LOG_PERIOD * 0.05 * 2) / (365.25 * 24 * 3600);
    double years_hi = 1e6 * (MONITOR_LOG_PERIOD_HI * 0.05 * 2) / (365.25 * 24 * 3600);
    printf("[info] head 寿命: 正常档 %.0f 天, 密节拍档 %.0f 天\n",
           years_lo * 365.25, years_hi * 365.25);
    check(years_lo * 365.25 > 600, "正常档 head 寿命 > 600 天（省寿命达标）");
    check(years_hi * 365.25 > 150, "密节拍档 head 寿命 > 150 天（趋势段可接受）");

    printf("\nPASS %d/FAIL %d\n", s_pass, s_fail);
    return (s_fail == 0) ? 0 : 1;
}