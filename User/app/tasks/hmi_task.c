/**
  * @file    hmi_task.c
  * @brief   人机交互任务：OLED 多页状态显示 + 按键交互 + LED 心跳
  *
  *  页面(KEY1 切换)：
  *    Page0 总览：安全态 / 报警 / 运行时长
  *    Page1 控制：速度 / 倾角 / PWM 占空比
  *    Page2 监测：气压(原始/滤波) / 报警等级
  *    Page3 定位：坐标 / 三基站距离 / 解算有效
  *   KEY2：清除安全锁定态(从 BRAKE/ERROR 恢复)
  */
#include "tasks.h"
#include "oled.h"
#include "Key.h"
#include "LED.h"
#include <string.h>

#define HMI_PERIOD_MS   (100)
#define HMI_PAGES       (4)

static const char* safety_str(SafetyState s)
{
    switch (s)
    {
        case SAFE_RUN:   return "RUN ";
        case SAFE_WARN:  return "WARN";
        case SAFE_BRAKE: return "BRK ";
        case SAFE_ERROR: return "ERR ";
        default:         return "??  ";
    }
}

static const char* alarm_str(AlarmLevel a)
{
    switch (a)
    {
        case ALARM_NORMAL: return "OK ";
        case ALARM_WARN:   return "WARN";
        case ALARM_CRIT:   return "CRIT";
        default:           return "??  ";
    }
}

void HMI_Task(void* pvParameters)
{
    (void)pvParameters;

    Health_Register(TASK_ID_HMI);

    OLED_Init();
    OLED_Clear();
    KEY_GPIO_Config();
    LED_GPIO_Config();

    uint8_t page = 0;
    uint32_t hb = 0;

    for (;;)
    {
        Health_Report(TASK_ID_HMI);

        /* --- 按键 --- */
        uint8_t key = Key_GetNum();
        if (key == 1) page = (page + 1) % HMI_PAGES;
        else if (key == 2) Control_ClearSafetyError();

        /* --- 读取全局状态 --- */
        SystemState snap;
        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                snap = g_state;
                xSemaphoreGive(xSystemStateMutex);
            }
            else memset(&snap, 0, sizeof(snap));
        }
        else memset(&snap, 0, sizeof(snap));

        /* --- 绘制页面 --- */
        OLED_Clear();
        switch (page)
        {
            case 0:
                OLED_ShowString(1, 1, "== OVERVIEW ==");
                OLED_ShowString(2, 1, "Safe:");
                OLED_ShowString(2, 7, (char*)safety_str(snap.safety));
                OLED_ShowString(3, 1, "Alm :");
                OLED_ShowString(3, 7, (char*)alarm_str(snap.alarm));
                OLED_ShowString(4, 1, "Up  :");
                OLED_ShowNum(4, 7, snap.uptime_sec, 6);
                break;
            case 1:
                OLED_ShowString(1, 1, "== CONTROL ==");
                OLED_ShowString(2, 1, "Spd:");
                OLED_ShowSignedFloat(2, 7, snap.speed, 6, 1);
                OLED_ShowString(3, 1, "Ang:");
                OLED_ShowSignedFloat(3, 7, snap.angle, 6, 1);
                OLED_ShowString(4, 1, "PWM:");
                OLED_ShowSignedNum(4, 7, snap.motor_duty, 4);
                break;
            case 2:
                OLED_ShowString(1, 1, "== MONITOR ==");
                OLED_ShowString(2, 1, "Raw:");
                OLED_ShowNum(2, 7, snap.pressure_raw, 4);
                OLED_ShowString(3, 1, "Bar:");
                OLED_ShowFloat(3, 7, snap.pressure_filt, 6, 2);
                OLED_ShowString(4, 1, "Alm:");
                OLED_ShowString(4, 7, (char*)alarm_str(snap.alarm));
                break;
            case 3:
                OLED_ShowString(1, 1, "== UWB ==");
                OLED_ShowString(2, 1, "X:");
                OLED_ShowFloat(2, 7, snap.position.x, 6, 2);
                OLED_ShowString(3, 1, "Y:");
                OLED_ShowFloat(3, 7, snap.position.y, 6, 2);
                OLED_ShowString(4, 1, snap.uwb_valid ? "LOCK" : "NOLOCK");
                break;
            default: break;
        }

        /* --- LED 心跳(绿) + 运行指示(蓝=控制环存活) --- */
        hb ^= 1;
        LED_Green(hb);
        LED_Blue(snap.safety == SAFE_RUN ? 1 : 0);

        vTaskDelay(pdMS_TO_TICKS(HMI_PERIOD_MS));
    }
}
