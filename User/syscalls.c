/**
  * @file    syscalls.c
  * @brief   C 库 I/O 重定向 + 堆初始化：Keil/ARMCC V5 与 GCC/newlib 双兼容
  *
  * - Keil (ARMCC V5, 完整 C 库, MicroLIB 关闭, 支持 printf %f):
  *     * #pragma __use_no_semihosting_swi 关闭半主机
  *     * 实现 _sys_* 系列, printf 最终经 _sys_write 重定向到 DBG_USART(USART2)
  *     * 实现 __user_setup_stackheap 给标准库提供独立堆(供 printf %f 临时缓冲)
  *       —— 与 FreeRTOS 的 ucHeap(heap_4) 互不干扰
  * - GCC / newlib-nano: 保留 _write / _sbrk 原实现(_write 由 bsp_usart.c 提供)
  * 不在此固化任何凭证。
  */
#include <stdint.h>

#ifdef __ARMCC_VERSION
/* ==================== Keil / ARMCC V5 路径 ==================== */
#include <stdio.h>
#include <rt_misc.h>
#include "stm32f10x.h"
#include "board_config.h"

#pragma import(__use_no_semihosting_swi)

/* 标准库堆：仅给 printf 等标准库内部 malloc 用；与 FreeRTOS ucHeap 相互独立 */
#define CLI_HEAP_SIZE  0x800u
static unsigned char s_cli_heap[CLI_HEAP_SIZE] __attribute__((aligned(8)));

struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;

/* 告知完整 C 库堆的基址/上限（栈由启动文件设定，此处不碰） */
__value_in_regs struct __initial_stackheap __user_setup_stackheap(
        unsigned R0, unsigned SP, unsigned R2, unsigned SL)
{
    struct __initial_stackheap config;
    config.heap_base  = (unsigned)&s_cli_heap[0];
    config.heap_limit = (unsigned)&s_cli_heap[CLI_HEAP_SIZE];
    return config;
}

/* 单字节阻塞发往调试串口（行为与原 _write 一致） */
static int cli_putc(int ch)
{
    USART_SendData(DBG_USART, (uint8_t)ch);
    while (USART_GetFlagStatus(DBG_USART, USART_FLAG_TXE) == RESET);
    return ch;
}

/* 兼容 MicroLIB（若被打开）以及直接调用 fputc 的代码 */
int fputc(int ch, FILE *f)
{
    (void)f;
    return cli_putc(ch);
}

int ferror(FILE *f) { (void)f; return EOF; }

void _sys_exit(int return_code) { (void)return_code; for (;;); }

int _sys_open(const char *name, int openmode)
{
    (void)name; (void)openmode;
    return 1;                       /* 所有文件映射到同一串行通道 */
}

int _sys_close(int fh) { (void)fh; return 0; }

int _sys_write(int fh, const unsigned char *buf, unsigned len, int mode)
{
    (void)fh; (void)mode;
    for (unsigned i = 0; i < len; i++) cli_putc(buf[i]);
    return (int)len;
}

int _sys_read(int fh, unsigned char *buf, unsigned len, int mode)
{
    (void)fh; (void)buf; (void)len; (void)mode;
    return 0;                       /* 不从标准输入阻塞等待 */
}

int _sys_istty(int fh) { (void)fh; return 1; }

int _sys_seek(int fh, long pos) { (void)fh; (void)pos; return -1; }

long _sys_flen(int fh) { (void)fh; return 0; }

#else
/* ==================== GCC / newlib-nano 路径 ==================== */
#include <sys/stat.h>
#include <errno.h>

extern char _end;        /* 由链接器提供：.bss 末尾 */
extern char _HeapLimit;  /* 由链接器提供：堆上限(栈之下) */

static char* s_heap = 0;

void* _sbrk(int incr)
{
    char* prev;
    if (s_heap == 0) s_heap = &_end;
    prev = s_heap;
    if ((s_heap + incr) > &_HeapLimit)
    {
        errno = ENOMEM;
        return (void*)-1;
    }
    s_heap += incr;
    return (void*)prev;
}

int _close(int fd)   { (void)fd; return -1; }
int _fstat(int fd, struct stat* st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd)  { (void)fd; return 1; }
off_t _lseek(int fd, off_t off, int whence) { (void)fd; (void)off; (void)whence; return 0; }
int _read(int fd, char* ptr, int len) { (void)fd; (void)ptr; (void)len; return 0; }

#endif /* __ARMCC_VERSION */
