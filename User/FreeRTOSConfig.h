#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ------------------------------------------------------------------------
 *  FreeRTOS V10.5.1 配置（适配 STM32F103RCT6 @ 72MHz, Cortex-M3）
 *  实时性：抢占式调度；20ms 控制节拍由硬件 TIM3 触发（见 scheduler.c）
 *  高稳定性：开启栈溢出钩子 / 内存分配失败钩子 / 断言
 * ---------------------------------------------------------------------- */

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     1
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 72000000 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 8 )
#define configMINIMAL_STACK_SIZE                ( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 24 * 1024 ) )
#define configMAX_TASK_NAME_LEN                ( 16 )
#define configUSE_TRACE_FACILITY                0
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/* 协同例程（不使用） */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/* 内核中断优先级（Cortex-M3 NVIC：0 最高, 15 最低） */
#define configKERNEL_INTERRUPT_PRIORITY         255   /* 15 << 4 */
/* 可在中断中安全调用 FreeRTOS API 的最低优先级（数值越大优先级越低） */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  11
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    191   /* 11 << 4 */
#define configLIBRARY_KERNEL_INTERRUPT_PRIORITY 15

/* 任务通知 / 互斥量 / 定时器 */
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               3
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            100

/* 统计与诊断（关闭运行时间统计以避免额外定时器依赖） */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_TASK_STATISTICS               1

/* 任务管理特性 */
#define configUSE_TASK_PRIORITIES               1
#define configUSE_TASK_SUSPEND                  1
#define configUSE_TASK_DELETION                 1
#define configUSE_TASK_PARAMETERTYPES           1

/* 栈溢出 & 内存失败钩子（高稳定性设计） */
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1

/* 断言（调试期捕获非法调用） */
#define configASSERT( x )    if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* API 包含控制 */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskCleanUpResources           0
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetSchedulerState          1

#endif /* FREERTOS_CONFIG_H */
