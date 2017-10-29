#include "infrared.h"


void IR_Init(){

     /* DMA controller clock enable */
     __HAL_RCC_DMA1_CLK_ENABLE();

     /* DMA interrupt init */
     /* DMA1_Channel1_IRQn interrupt configuration */
     HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
     HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

      ADC_ChannelConfTypeDef sConfig;

        /**Common config
        */
      hadc1.Instance = ADC1;
      hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
      hadc1.Init.ContinuousConvMode = ENABLE;
      hadc1.Init.DiscontinuousConvMode = DISABLE;
      hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
      hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
      hadc1.Init.NbrOfConversion = 4;
      if (HAL_ADC_Init(&hadc1) != HAL_OK)
      {
        Error_Handler();
      }

        /**Configure Regular Channel
        */
      sConfig.Channel = ADC_CHANNEL_6;
      sConfig.Rank = 1;
      sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }

        /**Configure Regular Channel
        */
      sConfig.Channel = ADC_CHANNEL_7;
      sConfig.Rank = 2;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }

        /**Configure Regular Channel
        */
      sConfig.Channel = ADC_CHANNEL_8;
      sConfig.Rank = 3;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }

        /**Configure Regular Channel
        */
      sConfig.Channel = ADC_CHANNEL_9;
      sConfig.Rank = 4;
      if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }

	  __enable_irq();
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)_adc_buf, 4);
      //HAL_ADC_Start_IT(&hadc1);
      //HAL_ADC_Start(&hadc1);
}

/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
    if(hadc->Instance == ADC1){
        val[0] = _adc_buf[0];
        val[1] = _adc_buf[1];
        val[2] = _adc_buf[2];
        val[3] = _adc_buf[3];
    }
}*/

int get_ir(ir_sensor_t ir){
    switch (ir){
        case IR_FL:
        //_sConfig.Channel = ADC_CHANNEL_6;
        return _adc_buf[3];
        break;

        case IR_FR:
        return _adc_buf[0];
        break;

        case IR_SL:
        return _adc_buf[2];
        break;

        case IR_SR:
        return _adc_buf[1];
        break;

        default:
        return 0;
        break;

    }
    /*if (HAL_ADC_ConfigChannel(&hadc1, &_sConfig) != HAL_OK){
        Error_Handler();
    }*/
    //return HAL_ADC_GetValue(&hadc1);
}
