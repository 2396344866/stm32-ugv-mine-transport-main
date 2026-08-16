#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "stm32f10x.h"

/* =========================================================================
 *  项目：STM32F103RCT6 工业物联网边缘节点 — 实时控制与监测系统
 *  说明：本文件为统一板级资源配置（引脚资源表 + 特性开关），
 *        所有子系统驱动/任务均从此处取引脚定义，避免分散硬编码。
 * ========================================================================= */

/* ----- 工程信息 ----- */
#define FW_NAME        "STM32F103RCT6_EdgeNode"
#define FW_VERSION     "1.0.0"

/* ----- 特性开关（便于裁剪与扩展） ----- */
#define USE_RTOS                1   /* FreeRTOS 抢占式任务调度            */
#define USE_CONTROL_SUBSYS      1   /* 双闭环 PID 实时控制               */
#define USE_MONITOR_SUBSYS      1   /* 低功耗气压监测                    */
#define USE_RTC_DEPENDENT       1   /* 监测子系统依赖 RTC 时标           */
#define USE_UWB_SUBSYS          1   /* UWB 无线定位                      */
#define USE_HMI_SUBSYS          1   /* OLED / Key / LED                 */
#define USE_ESP8266             1   /* ESP8266 WiFi 数据上传            */
#define USE_AT24C256            1   /* AT24C256 EEPROM 本地存储         */
#define USE_MPU6050_DMP         1   /* MPU6050 DMP 姿态解算             */
#define UWB_USE_DW3000_SPI      0   /* 0:BU03(UART-AT) 路径; 1:DW3000 直驱SPI路径(休眠, 未编译) */

/* ----- 实时性 ----- */
#define CONTROL_PERIOD_MS       20  /* 20ms 确定性控制节拍              */

/* =========================================================================
 *  引脚资源表 (STM32F103RCT6, LQFP64, HD, 256KB Flash / 48KB RAM)
 * ========================================================================= */

/* --- 控制子系统（双闭环 PID / 电机 / 编码器 / 状态指示） --- */
#define MOTOR_PWM_PORT      GPIOA
#define MOTOR_PWM_PIN       GPIO_Pin_8      /* TIM1_CH1 电机 PWM          */
#define MOTOR_DIR_A_PORT    GPIOB
#define MOTOR_DIR_A_PIN     GPIO_Pin_12     /* H桥方向 A                  */
#define MOTOR_DIR_B_PORT    GPIOB
#define MOTOR_DIR_B_PIN     GPIO_Pin_13     /* H桥方向 B                  */
#define ENCODER_TIM         TIM2
#define ENCODER_PORT        GPIOA
#define ENCODER_A_PIN       GPIO_Pin_0      /* 编码器 A (TI1)             */
#define ENCODER_B_PIN       GPIO_Pin_1      /* 编码器 B (TI2)             */
#define STATUS_WARN_PORT    GPIOB
#define STATUS_WARN_PIN     GPIO_Pin_0      /* 黄灯：一级预警             */
#define STATUS_BUZ_PORT     GPIOB
#define STATUS_BUZ_PIN      GPIO_Pin_1      /* 蜂鸣器                     */
#define STATUS_ERR_PORT     GPIOB
#define STATUS_ERR_PIN      GPIO_Pin_2      /* 红灯：故障/刹车锁死        */

/* --- HMI 子系统 --- */
#define OLED_SCL_PORT       GPIOB
#define OLED_SCL_PIN        GPIO_Pin_8      /* OLED I2C 模拟 SCL          */
#define OLED_SDA_PORT       GPIOB
#define OLED_SDA_PIN        GPIO_Pin_9      /* OLED I2C 模拟 SDA          */
#define KEY1_PORT           GPIOC
#define KEY1_PIN            GPIO_Pin_0
#define KEY2_PORT           GPIOC
#define KEY2_PIN            GPIO_Pin_1
#define LED_BLUE_PORT       GPIOC
#define LED_BLUE_PIN        GPIO_Pin_13
#define LED_GREEN_PORT      GPIOC
#define LED_GREEN_PIN       GPIO_Pin_14

/* --- 监测子系统（工业管道气压） --- */
#define PRESSURE_ADC        ADC1
#define PRESSURE_ADC_CH     ADC_Channel_4   /* PA4, 扩散硅压力变送器      */
#define PRESSURE_GPIO_PORT  GPIOA
#define PRESSURE_GPIO_PIN   GPIO_Pin_4

/* --- UWB 定位子系统（安信可 BU03 模块, UART-AT） --- */
#define UWB_USART           USART3
#define UWB_USART_CLK       RCC_APB1Periph_USART3
#define UWB_GPIO_CLK        RCC_APB2Periph_GPIOC
#define UWB_TX_PORT         GPIOC
#define UWB_TX_PIN          GPIO_Pin_10     /* USART3 部分重映射 TX       */
#define UWB_RX_PORT         GPIOC
#define UWB_RX_PIN          GPIO_Pin_11     /* USART3 部分重映射 RX       */
#define UWB_USART_IRQn      USART3_IRQn
#define UWB_USART_BAUD      115200

/* --- 通信子系统（ESP8266 WiFi 上行） --- */
#define ESP_USART           USART1
#define ESP_GPIO_CLK        RCC_APB2Periph_GPIOA
#define ESP_TX_PORT         GPIOA
#define ESP_TX_PIN          GPIO_Pin_9
#define ESP_RX_PORT         GPIOA
#define ESP_RX_PIN          GPIO_Pin_10
#define ESP_USART_IRQn      USART1_IRQn
#define ESP_USART_BAUD      115200

/* --- 调试控制台（USART2） --- */
#define DBG_USART           USART2
#define DBG_GPIO_CLK        RCC_APB2Periph_GPIOA
#define DBG_TX_PORT         GPIOA
#define DBG_TX_PIN          GPIO_Pin_2
#define DBG_RX_PORT         GPIOA
#define DBG_RX_PIN          GPIO_Pin_3
#define DBG_USART_IRQn      USART2_IRQn
#define DBG_USART_BAUD      115200

/* --- MPU6050 DMP 姿态（I2C2, PB10/PB11, 定义见 MPU6050_Reg.h） --- */
#define MPU_I2C             I2C2

/* --- AT24C256 EEPROM（I2C2 共享总线, 地址 0xAE = 0x57<<1） --- */
#define AT24C256_ADDR       0xAE

#endif /* BOARD_CONFIG_H */
