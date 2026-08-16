/**
  * @file    bsp_esp8266.c
  * @brief   ESP8266 WiFi 驱动（AT 指令，USART1）
  *
  *  流程：AT 握手 -> 设 Station 模式 -> 连接 AP -> 建立 TCP。
  *  注：ssid/pwd/host 为占位，部署时由配置注入，不直接固化敏感凭证。
  */
#include "bsp_esp8266.h"
#include "bsp_usart.h"
#include "board_config.h"
#include <string.h>
#include <stdio.h>

void BSP_ESP8266_Init(void)
{
    BSP_USART_Init(ESP_USART, ESP_USART_BAUD, 0);
}

int ESP8266_AT_Check(const char* expected, uint32_t timeout_ms)
{
    uint8_t buf[128];
    int n = BSP_USART_Read(ESP_USART, buf, sizeof(buf) - 1, timeout_ms);
    if (n <= 0) return -1;
    buf[n] = 0;
    return (strstr((char*)buf, expected) != NULL) ? 0 : -1;
}

int ESP8266_ConnectWiFi(const char* ssid, const char* pwd)
{
    char cmd[96];
    BSP_USART_SendString(ESP_USART, "AT\r\n");
    if (ESP8266_AT_Check("OK", 1000) != 0) return -1;

    BSP_USART_SendString(ESP_USART, "AT+CWMODE=1\r\n");
    if (ESP8266_AT_Check("OK", 1000) != 0) return -1;

    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pwd);
    BSP_USART_SendString(ESP_USART, cmd);
    return ESP8266_AT_Check("OK", 8000);
}

int ESP8266_OpenTCP(const char* host, const char* port)
{
//    char cmd[96];
    BSP_USART_SendString(ESP_USART, "AT+CIPSTART=\"TCP\",\"");
    BSP_USART_SendString(ESP_USART, host);
    BSP_USART_SendString(ESP_USART, "\",");
    BSP_USART_SendString(ESP_USART, port);
    BSP_USART_SendString(ESP_USART, "\r\n");
    return ESP8266_AT_Check("OK", 5000);
}

int ESP8266_Send(const uint8_t* data, uint16_t len)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", len);
    BSP_USART_SendString(ESP_USART, cmd);
    if (ESP8266_AT_Check(">", 2000) != 0) return -1;
    BSP_USART_Send(ESP_USART, data, len);
    return ESP8266_AT_Check("SEND OK", 3000);
}
