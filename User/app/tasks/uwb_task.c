/**
  * @file    uwb_task.c
  * @brief   UWB 无线定位任务：BU03(TWR) 测距 -> 三基站二维三边定位
  *
  *  流程：周期性向三个固定基站发起测距，取得距离后调用
  *        UWB_Trilaterate2D(Gauss-Newton 最小二乘) 解算标签二维坐标。
  *  测距硬件为集成安信可 BU03 模组（基于 DW3000），主机通过 UART-AT 交互，
  *  测距在模块内部以 TWR 完成，主机仅解析距离结果帧。
  */
#include "tasks.h"
#include "bsp_uwb.h"
#include "uwb_2d.h"

/* 三基站固定坐标(米)，按现场部署标定 */
static const Point2D s_anchors[3] =
{
    { 0.0f,  0.0f },
    { 6.0f,  0.0f },
    { 3.0f,  5.0f }
};

#define UWB_PERIOD_MS   (200)   /* 定位周期 */
#define UWB_RETRY        (3)

void UWB_Task(void* pvParameters)
{
    (void)pvParameters;

    Health_Register(TASK_ID_UWB);

#if USE_UWB_SUBSYS
    BSP_UWB_Init();

    float d[3];
    Point2D pos = {0, 0};

    for (;;)
    {
        Health_Report(TASK_ID_UWB);

        uint8_t ok = 1;
        for (int i = 0; i < 3; i++)
        {
            int got = 0;
            for (int r = 0; r < UWB_RETRY && !got; r++)
            {
                if (UWB_StartRanging() == 0)
                {
                    if (UWB_ReadDistance(&d[i]) == 0) got = 1;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            if (!got)
            {
                ok = 0;
                Error_Record(ERR_UWB_NO_RESPONSE);
                break;
            }
        }

        uint8_t valid = 0;
        if (ok)
        {
            if (UWB_Trilaterate2D(s_anchors, d, &pos) == 0) valid = 1;
        }

        if (xSystemStateMutex != NULL)
        {
            if (xSemaphoreTake(xSystemStateMutex, 0) == pdPASS)
            {
                if (valid)
                {
                    g_state.position = pos;
                    g_state.uwb_d[0] = d[0];
                    g_state.uwb_d[1] = d[1];
                    g_state.uwb_d[2] = d[2];
                }
                g_state.uwb_valid = valid;
                xSemaphoreGive(xSystemStateMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(UWB_PERIOD_MS));
    }
#else
    for (;;)
    {
        Health_Report(TASK_ID_UWB);
        vTaskDelay(pdMS_TO_TICKS(UWB_PERIOD_MS));
    }
#endif
}
