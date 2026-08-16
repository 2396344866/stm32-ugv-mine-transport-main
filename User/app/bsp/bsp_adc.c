/**
  * @file    bsp_adc.c
  * @brief   气压监测 ADC 采集（PA4, 单通道软件触发）
  */
#include "bsp_adc.h"
#include "board_config.h"

void BSP_ADC_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    ADC_InitTypeDef         ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);   /* 12MHz ADC 时钟 */

    GPIO_InitStructure.GPIO_Pin  = PRESSURE_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(PRESSURE_GPIO_PORT, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Mode             = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode     = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign        = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel     = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);

    /* ADC 校准 */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

uint16_t BSP_ADC_ReadRaw(void)
{
    ADC_RegularChannelConfig(ADC1, PRESSURE_ADC_CH, 1, ADC_SampleTime_55Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return ADC_GetConversionValue(ADC1);
}
