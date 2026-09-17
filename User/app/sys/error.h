#ifndef ERROR_H
#define ERROR_H

#include <stdint.h>

/* 系统错误码（高稳定性设计：统一错误记录，供健康管理任务消费） */
typedef enum
{
    ERR_NONE             = 0,
    ERR_ADC_READ         = 1,
    ERR_I2C_TIMEOUT      = 2,
    ERR_EEPROM_WRITE     = 3,
    ERR_UWB_NO_RESPONSE  = 4,
    ERR_ESP_AT           = 5,
    ERR_MPU_DMP          = 6,
    ERR_RTC_CONFIG       = 7,
    ERR_WATCHDOG_RESET   = 8,
    ERR_CAN_TX_TIMEOUT   = 9,   /* 门锁指令重发达上限仍未收到回执 */
    ERR_CAN_BUSOFF       = 10,  /* CAN 总线关闭（Bus-Off） */
    ERR_CAN_INIT         = 11,  /* bxCAN 初始化失败 */
    ERR_UNKNOWN          = 0xFF
} SysErrCode;

/* 可与数值互转的错误码总数（供错误计数数组定长） */
#define ERR_CODE_COUNT      (12)

void     Error_Record(SysErrCode code);
uint32_t Error_CountTotal(void);
uint32_t Error_CountCode(SysErrCode code);
SysErrCode Error_Last(void);
void     Error_Clear(void);

#endif /* ERROR_H */
