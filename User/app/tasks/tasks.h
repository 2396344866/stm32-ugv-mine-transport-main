#ifndef TASKS_H
#define TASKS_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

#include "board_config.h"
#include "alarm.h"
#include "safety_fsm.h"
#include "uwb_2d.h"
#include "health.h"
#include "error.h"

/* =========================================================================
 *  任务层公共头：任务 ID、优先级、全局系统状态（跨任务共享）
 *  设计：控制环为最高优先级实时任务；监测/定位/通信/HMI 依次降级；
 *        健康检查与看门狗喂狗独立成任务，保障"高稳定性"要求。
 * ========================================================================= */

/* 任务 ID（用于健康管理注册，需 < HEALTH_MAX_TASKS） */
typedef enum
{
    TASK_ID_CONTROL   = 0,
    TASK_ID_MONITOR   = 1,
    TASK_ID_UWB       = 2,
    TASK_ID_HMI       = 3,
    TASK_ID_COMM      = 4,
    TASK_ID_HOUSEKEEP = 5,
    TASK_ID_BCM       = 6        /* 车身 CAN：门锁指令下发与回执处理 */
} TaskId;

/* 任务优先级（configMAX_PRIORITIES = 8，数值越大优先级越高） */
#define TASK_PRIO_CONTROL    (5)   /* 20ms 确定性控制环，实时性最高 */
#define TASK_PRIO_HOUSEKEEP  (4)   /* 喂狗 + 健康巡检 */
#define TASK_PRIO_MONITOR    (3)   /* 气压监测 + 低功耗调度 */
#define TASK_PRIO_UWB        (2)   /* UWB 定位 */
#define TASK_PRIO_COMM       (2)   /* WiFi/USB 上行 */
#define TASK_PRIO_HMI        (1)   /* OLED/Key/LED 人机交互 */
#define TASK_PRIO_BCM        (3)   /* 车身 CAN 门锁指令（与监测同级，低于控制） */

#define TASK_STACK_CONTROL   (256)
#define TASK_STACK_DEFAULT   (192)
#define TASK_STACK_BCM       (256)  /* snprintf 调试行，栈留余量 */
#define TASK_STACK_HMI       (256)

/* 全局系统状态（被多个任务读写，访问需持 xSystemStateMutex） */
typedef struct
{
    /* 控制子系统 */
    float   speed;          /* 当前速度(编码器估计, 单位 pulse/s 折算) */
    float   angle;          /* 当前倾角(度, MPU6050 pitch) */
    float   tension;        /* 张力/负载代理值(N) */
    int16_t motor_duty;     /* 当前 PWM 占空比 0..999 */
    SafetyState safety;     /* 安全状态机状态 */
    uint32_t control_ticks; /* 控制环累计节拍数 */

    /* 监测子系统 */
    uint16_t pressure_raw;  /* ADC 原始值 0..4095 */
    float    pressure_bar;  /* 折算气压(bar) */
    float    pressure_filt; /* 滤波后气压 */
    AlarmLevel alarm;       /* 报警等级 */

    /* 定位子系统 */
    Point2D position;       /* 二维解算坐标(米) */
    float   uwb_d[3];       /* 三基站测量距离(米) */
    uint8_t uwb_valid;      /* 解算有效性 */

    /* 车身 CAN 子系统（BCM ↔ 车门控制器） */
    uint8_t  door_lock_mask;  /* 最近一次回执回报的门锁到位位图 */
    uint8_t  door_cmd_busy;   /* 有在途门锁指令（等待回执/重试中） */

    /* 系统级 */
    uint8_t  lowpower_mode; /* 低功耗(STOP)请求标志 */
    uint32_t uptime_sec;    /* 运行时长(秒) */
    uint32_t err_total;     /* 累计错误数 */
    SysErrCode last_err;    /* 最近一次错误码 */
} SystemState;

extern SystemState        g_state;
extern SemaphoreHandle_t  xSystemStateMutex;
extern QueueHandle_t      xLogQueue;      /* 监测记录队列(供通信任务上传) */
extern QueueHandle_t      xDoorCmdQueue;  /* 门锁请求队列(供其它任务投递给 BCM 任务) */

/* 控制任务对外接口（供 HMI 等人机交互清除锁定态） */
void Control_ClearSafetyError(void);

/* BCM 任务对外接口：投递一条门锁请求（任务上下文调用，不阻塞） */
int  BCM_RequestDoorLock(uint8_t door_mask, uint8_t action);

/* 各任务入口 */
void Control_Task(void* pvParameters);
void Monitor_Task(void* pvParameters);
void UWB_Task(void* pvParameters);
void HMI_Task(void* pvParameters);
void Comm_Task(void* pvParameters);
void Housekeep_Task(void* pvParameters);
void BCM_Task(void* pvParameters);

#endif /* TASKS_H */
