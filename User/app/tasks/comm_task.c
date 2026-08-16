/**
  * @file    comm_task.c
  * @brief   通信任务：ESP8266 WiFi 上行 + 调试控制台(RS232/USB-CDC)镜像
  *
  *  上行内容：系统状态快照(周期) + 监测落盘记录(来自 xLogQueue)。
  *  安全：WiFi 凭证以编译期占位符提供，不内嵌任何真实密钥
  *        （原 demo 中的 Aliyun MQTT 明文口令已移除）。
  *  健壮性：WiFi/TCP 不可用时自动降级为仅调试串口输出，不阻塞系统。
  */
#include "tasks.h"
#include "bsp_esp8266.h"
#include "bsp_usart.h"
#include <string.h>
#include <stdio.h>

/* 编译期占位符：部署时替换为现场 AP 与服务器（请勿提交真实密钥） */
#ifndef COMM_WIFI_SSID
#define COMM_WIFI_SSID   "YOUR_SSID"
#endif
#ifndef COMM_WIFI_PWD
#define COMM_WIFI_PWD    "YOUR_PASSWORD"
#endif
#ifndef COMM_HOST
#define COMM_HOST        "192.168.1.100"
#endif
#ifndef COMM_PORT
#define COMM_PORT        "8080"
#endif

#define COMM_PERIOD_MS   (1000)

void Comm_Task(void* pvParameters)
{
    (void)pvParameters;

    Health_Register(TASK_ID_COMM);

    uint8_t wifi_ok = 0;

#if USE_ESP8266
    BSP_ESP8266_Init();
    if (ESP8266_ConnectWiFi(COMM_WIFI_SSID, COMM_WIFI_PWD) == 0)
    {
        if (ESP8266_OpenTCP(COMM_HOST, COMM_PORT) == 0)
            wifi_ok = 1;
        else
            Error_Record(ERR_ESP_AT);
    }
    else
    {
        Error_Record(ERR_ESP_AT);
    }
#endif

    uint8_t logbuf[8];
    char line[96];

    for (;;)
    {
        Health_Report(TASK_ID_COMM);

        /* 读取全局状态并生成上报行 */
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

        int len = snprintf(line, sizeof(line),
            "S%d A%d P%.2f V%.1f X%.2f Y%.2f E%lu\r\n",
            snap.safety, snap.alarm, snap.pressure_filt,
            snap.speed, snap.position.x, snap.position.y, snap.err_total);

        /* 调试控制台镜像（printf -> DBG_USART，始终可用） */
        BSP_USART_SendString(DBG_USART, line);

        /* WiFi 上行（仅在已建立 TCP 时） */
        if (wifi_ok)
        {
            if (ESP8266_Send((const uint8_t*)line, (uint16_t)len) != 0)
            {
                Error_Record(ERR_ESP_AT);
                wifi_ok = 0;   /* 连接断开，下次周期尝试重连 */
            }
        }

        /* 排空监测日志队列，逐条上行 */
        while (xLogQueue != NULL && xQueueReceive(xLogQueue, logbuf, 0) == pdPASS)
        {
            char l[64];
            int n = snprintf(l, sizeof(l),
                "LOG %02d:%02d:%02d A%d R%u F%u\r\n",
                logbuf[0], logbuf[1], logbuf[2], logbuf[3],
                (uint16_t)(logbuf[4] | (logbuf[5] << 8)),
                (uint16_t)(logbuf[6] | (logbuf[7] << 8)));
            BSP_USART_SendString(DBG_USART, l);
            if (wifi_ok) ESP8266_Send((const uint8_t*)l, (uint16_t)n);
        }

        vTaskDelay(pdMS_TO_TICKS(COMM_PERIOD_MS));
    }
}
