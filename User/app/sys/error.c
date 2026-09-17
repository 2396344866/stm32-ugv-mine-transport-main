/**
  * @file    error.c
  * @brief   统一错误记录（轻量级，无动态分配）
  */
#include "error.h"

#define ERR_HISTORY_DEPTH  16

static uint32_t    s_total        = 0;
static SysErrCode  s_last         = ERR_NONE;
static uint32_t    s_code_count[ERR_CODE_COUNT] = {0};
static SysErrCode  s_history[ERR_HISTORY_DEPTH];
static uint8_t     s_hist_idx     = 0;

void Error_Record(SysErrCode code)
{
    if (code == ERR_NONE) return;

    s_total++;
    s_last = code;
    if ((int)code < ERR_CODE_COUNT) s_code_count[(int)code]++;

    s_history[s_hist_idx] = code;
    s_hist_idx = (s_hist_idx + 1) % ERR_HISTORY_DEPTH;
}

uint32_t Error_CountTotal(void) { return s_total; }
uint32_t Error_CountCode(SysErrCode code)
{
    return ((int)code < ERR_CODE_COUNT) ? s_code_count[(int)code] : 0;
}
SysErrCode Error_Last(void) { return s_last; }
void Error_Clear(void)
{
    int i;
    s_total = 0;
    s_last  = ERR_NONE;
    for (i = 0; i < ERR_CODE_COUNT; i++) s_code_count[i] = 0;
}
