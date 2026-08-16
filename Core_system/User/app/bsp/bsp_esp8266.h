#ifndef BSP_ESP8266_H
#define BSP_ESP8266_H

#include "stm32f10x.h"

/* ESP8266 WiFi 上行（USART1, AT 指令），用于监测数据云端上传 */
void BSP_ESP8266_Init(void);
int  ESP8266_AT_Check(const char* expected, uint32_t timeout_ms);
int  ESP8266_ConnectWiFi(const char* ssid, const char* pwd);
int  ESP8266_OpenTCP(const char* host, const char* port);
int  ESP8266_Send(const uint8_t* data, uint16_t len);

#endif /* BSP_ESP8266_H */
