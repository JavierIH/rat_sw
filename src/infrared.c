#include "infrared.h"


void IR_Init(){
    ADC_ChannelConfTypeDef sConfig;

    // Common config
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    //Configure Regular Channel
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_ADC_Start(&hadc1);
}

int get_ir(ir_sensor_t ir){
    switch (ir){
        case IR_FL:
        _sConfig.Channel = ADC_CHANNEL_6;
        break;

        case IR_FR:
        _sConfig.Channel = ADC_CHANNEL_7;
        break;

        case IR_SL:
        _sConfig.Channel = ADC_CHANNEL_8;
        break;

        case IR_SR:
        _sConfig.Channel = ADC_CHANNEL_9;
        break;

        default:
        return 0;
        break;

    }
    if (HAL_ADC_ConfigChannel(&hadc1, &_sConfig) != HAL_OK){
        Error_Handler();
    }
    return HAL_ADC_GetValue(&hadc1);
}
