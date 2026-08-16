/**
  * @file    error.c
  * @brief   统一错误记录（轻量级，无动态分配）
  */
#include "error.h"

#define ERR_HISTORY_DEPTH  16

static uint32_t    s_total        = 0;
static SysErrCode  s_last         = ERR_NONE;
static uint32_t    s_code_count[9] = {0};
static SysErrCode  s_history[ERR_HISTORY_DEPTH];
static uint8_t     s_hist_idx     = 0;

void Error_Record(SysErrCode code)
{
    if (code == ERR_NONE) return;

    s_total++;
    s_last = code;
    if (code <= 8) s_code_count[code]++;

    s_history[s_hist_idx] = code;
    s_hist_idx = (s_hist_idx + 1) % ERR_HISTORY_DEPTH;
}

uint32_t Error_CountTotal(void) { return s_total; }
uint32_t Error_CountCode(SysErrCode code) { return (code <= 8) ? s_code_count[code] : 0; }
SysErrCode Error_Last(void) { return s_last; }
void Error_Clear(void) { s_total = 0; s_last = ERR_NONE; for (int i = 0; i <= 8; i++) s_code_count[i] = 0; }
