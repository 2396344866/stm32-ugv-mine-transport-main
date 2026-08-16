/**
  * @file    control_task.c
  * @brief   实时控制任务：20ms 确定性节拍下的双闭环级联 PID + 安全状态机
  *
  *  结构（以"抗扰动速度/姿态控制"为模型）：
  *   外环(姿态 PD)：倾角误差 -> 期望速度（姿态恢复）
  *   内环(速度 PI)：期望速度 -> PWM 占空比（增量式，抗饱和）
  *   安全状态机：速度/倾角/张力越限进入 WARN/BRAKE/ERROR（后两者锁定）。
  *
  *  与硬件解耦：节拍由 scheduler.c 的 TIM3 中断释放 xControlSemaphore 驱动，
  *  本任务在任务上下文完成全部浮点运算，符合中断底半部原则。
  */
#include "tasks.h"
#include "scheduler.h"
#include "pid.h"
#include "filter.h"
#include "bsp_motor.h"
#include "bsp_watchdog.h"
#include "MPU6050.h"     /* MPU6050_DMP_GetData（USE_MPU6050_DMP 分支） */

/* ---- 控制参数（可按项目整定，集中管理便于调参） ---- */
#define CTRL_TARGET_ANGLE      (0.0f)    /* 期望倾角(度)：直立 */
#define CTRL_ANGLE_KP          (5.0f)    /* 外环姿态比例 */
#define CTRL_ANGLE_KD          (2.0f)    /* 外环姿态微分(角速度阻尼) */
#define CTRL_SPEED_KP          (12.0f)   /* 内环速度比例 */
#define CTRL_SPEED_KI          (0.5f)    /* 内环速度积分 */
#define CTRL_SPEED_OUT_MAX     (999.0f)  /* PWM 上限 */
#define CTRL_SPEED_OUT_MIN     (-999.0f)
#define CTRL_INTEG_LIMIT       (300.0f)  /* 积分限幅(抗饱和) */
#define CTRL_ENC_TO_SPEED      (1.0f)    /* 编码器计数增量 -> 速度系数 */
#define CTRL_ANGLE_RATE_GAIN   (1.0f)    /* 角速度 = (angle - angle_prev)/dt 增益 */

/* 安全阈值（与 Safety_Init 保持一致） */
#define SAFE_TARGET_SPEED (60.0f)
#define SAFE_WARN_SPEED   (90.0f)
#define SAFE_MAX_SPEED    (120.0f)
#define SAFE_CRIT_ANGLE   (25.0f)
#define SAFE_MAX_TENSION  (200.0f)
#define SAFE_MIN_TENSION  (5.0f)

static PID_Handle s_speed_pid;
static SafetyFSM  s_safety;

/* 供 HMI 任务清除 BRAKE/ERROR 锁定态 */
void Control_ClearSafetyError(void)
{
    Safety_ClearError(&s_safety);
}
static float      s_angle_prev = 0.0f;
static float      s_duty = 0.0f;   /* 增量式累加后的占空比 */

void Control_Task(void* pvParameters)
{
    (void)pvParameters;

    Health_Register(TASK_ID_CONTROL);

    /* 内环速度 PID（增量式） */
    PID_Init(&s_speed_pid, CTRL_SPEED_KP, CTRL_SPEED_KI, 0.0f,
             CTRL_SPEED_OUT_MAX, CTRL_SPEED_OUT_MIN, CTRL_INTEG_LIMIT);
    s_speed_pid.mode = 1;  /* 增量式 */

    /* 安全状态机 */
    Safety_Init(&s_safety, SAFE_TARGET_SPEED, SAFE_WARN_SPEED, SAFE_MAX_SPEED,
                SAFE_CRIT_ANGLE, SAFE_MAX_TENSION, SAFE_MIN_TENSION);

    BSP_Encoder_Reset();

    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

    for (;;)
    {
        /* 等待 20ms 控制节拍（TIM3 中断释放信号量） */
        if (xSemaphoreTake(xControlSemaphore, pdMS_TO_TICKS(40)) != pdPASS)
        {
            /* 超时说明节拍中断异常，记录但不退出 */
            Error_Record(ERR_UNKNOWN);
            Health_Report(TASK_ID_CONTROL);
            continue;
        }

        Health_Report(TASK_ID_CONTROL);

        /* --- 1. 采样反馈 --- */
        int16_t enc_delta = BSP_Encoder_ReadDelta();        /* 本周期计数增量 */
        float speed = (float)enc_delta * CTRL_ENC_TO_SPEED; /* 速度估计 */

#if USE_MPU6050_DMP
        static uint8_t mpu_ok = 1;
        if (MPU6050_DMP_GetData(&pitch, &roll, &yaw) != 0)
        {
            if (mpu_ok) { Error_Record(ERR_MPU_DMP); mpu_ok = 0; }
        }
        else mpu_ok = 1;
#endif
        float angle = pitch;
        float angle_rate = (angle - s_angle_prev) / (CONTROL_PERIOD_MS / 1000.0f);
        s_angle_prev = angle;

        /* 张力代理值：由监测子系统气压折算（项目特定传感器可替换） */
        float tension = 0.0f;
        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                tension = g_state.tension;
                xSemaphoreGive(xSystemStateMutex);
            }
        }

        /* --- 2. 安全状态机（先于控制生效，越限即动作） --- */
        SafetyState st = Safety_Update(&s_safety, speed, angle, tension);

        /* --- 3. 级联 PID --- */
        float target_speed = 0.0f;
        float duty_out = s_duty;

        if (st == SAFE_RUN || st == SAFE_WARN)
        {
            /* 外环(姿态 PD)：倾角误差 -> 期望速度 */
            float angle_err = CTRL_TARGET_ANGLE - angle;
            target_speed = CTRL_ANGLE_KP * angle_err
                         - CTRL_ANGLE_KD * angle_rate * CTRL_ANGLE_RATE_GAIN;

            /* 限制期望速度在安全范围内 */
            if (target_speed >  SAFE_MAX_SPEED) target_speed =  SAFE_MAX_SPEED;
            if (target_speed < -SAFE_MAX_SPEED) target_speed = -SAFE_MAX_SPEED;

            /* 内环(速度 PI 增量式)：期望速度 -> PWM 增量 */
            PID_SetTarget(&s_speed_pid, target_speed);
            float d_duty = PID_Calc_Incremental(&s_speed_pid, speed);
            duty_out += d_duty;

            /* 占空比限幅 */
            if (duty_out >  CTRL_SPEED_OUT_MAX) duty_out =  CTRL_SPEED_OUT_MAX;
            if (duty_out <  CTRL_SPEED_OUT_MIN) duty_out =  CTRL_SPEED_OUT_MIN;
        }
        else
        {
            /* BRAKE / ERROR：锁定输出为 0（刹车），需显式清除 */
            duty_out = 0.0f;
            s_duty = 0.0f;
            PID_Reset(&s_speed_pid);
        }
        s_duty = duty_out;

        /* --- 4. 执行机构 --- */
        int16_t duty_i = (int16_t)duty_out;
        if (duty_i < 0) duty_i = -duty_i;             /* 占空比取绝对值，方向另控 */
        if (duty_i > 999) duty_i = 999;
        BSP_Motor_SetDuty((uint16_t)duty_i);
        BSP_Motor_SetDir((duty_out < 0.0f) ? MOTOR_REV : MOTOR_FWD);

        /* 状态指示：WARN->黄灯; ERROR/BRAKE->红灯; 正常->灭 */
        uint8_t warn = (st == SAFE_WARN) ? 1 : 0;
        uint8_t err  = (st == SAFE_ERROR || st == SAFE_BRAKE) ? 1 : 0;
        BSP_Status_Set(warn, 0, err);

        /* --- 5. 回写全局状态 --- */
        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                g_state.speed         = speed;
                g_state.angle         = angle;
                g_state.tension       = tension;
                g_state.motor_duty    = (int16_t)duty_out;
                g_state.safety        = st;
                g_state.control_ticks++;
                xSemaphoreGive(xSystemStateMutex);
            }
        }
    }
}
