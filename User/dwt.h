/**
  * @file    dwt.h
  * @brief   Cortex-M3 DWT 数据观察点与跟踪单元寄存器定义
  *
  * CMSIS 3.5.1 的 core_cm3.h 仅定义了 CoreDebug(DEMCR)，未提供 DWT
  * 寄存器结构。此头文件补齐工程所需的 DWT 访问（CYCCNT 周期计数器）。
  * 基址 0xE0001000，仅需使用 CTRL / CYCCNT 两个寄存器。
  */
#ifndef __DWT_H
#define __DWT_H

#include <stdint.h>

#ifndef DWT_CTRL_CYCCNTENA_Msk
#define DWT_CTRL_CYCCNTENA_Msk    (1UL << 0)   /* CYCCNT 使能位 */
#endif

typedef struct
{
    volatile uint32_t CTRL;       /* 0x00 控制寄存器 */
    volatile uint32_t CYCCNT;     /* 0x04 周期计数寄存器 */
    volatile uint32_t CPICNT;     /* 0x08 指令周期计数 */
    volatile uint32_t EXCCNT;     /* 0x0C 异常开销计数 */
    volatile uint32_t SLEEPCNT;   /* 0x10 休眠计数 */
    volatile uint32_t LSUCNT;     /* 0x14 加载/存储单元计数 */
    volatile uint32_t FOLDCNT;    /* 0x18 分支折叠计数 */
    volatile uint32_t PCSR;       /* 0x1C 程序计数采样 */
    volatile uint32_t COMP0;      /* 0x20 比较器 0 */
    volatile uint32_t MASK0;      /* 0x24 */
    volatile uint32_t FUNCTION0;  /* 0x28 */
    uint32_t RESERVED0;
    volatile uint32_t COMP1;      /* 0x30 比较器 1 */
    volatile uint32_t MASK1;      /* 0x34 */
    volatile uint32_t FUNCTION1;  /* 0x38 */
} DWT_Type;

#define DWT_BASE    (0xE0001000UL)
#define DWT         ((DWT_Type *)DWT_BASE)

#endif /* __DWT_H */
