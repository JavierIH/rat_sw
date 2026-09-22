#include "infrared.h"
#include "error.h"

volatile uint32_t _adc_buf[4];
DMA_HandleTypeDef hdma_adc1;
ADC_HandleTypeDef hadc1;

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
      if(HAL_ADC_Start_DMA(&hadc1, (uint32_t *)_adc_buf, 4) != HAL_OK){
        Error_Handler();
      }
      //HAL_ADC_Start_IT(&hadc1);
      //HAL_ADC_Start(&hadc1);
}

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


//-0.00000002083*x*x*x + 0.0001119*x*x - 0.2135*x + 185;
float get_ir_mm(ir_sensor_t ir){
    float x = get_ir(ir);
    if(ir == IR_SL){
        return -0.00000005219*x*x*x +0.0002629*x*x -0.4566*x +325.6;
    }
    else if(ir == IR_SR){
        return -0.00000003241*x*x*x +0.0001505*x*x -0.25*x +189;
    }
    else if(ir == IR_FL){
        return -0.00000002278*x*x*x +0.000132*x*x -0.2627*x +237.7;
    }
    else if(ir == IR_FR){
        return -0.00000003535*x*x*x +0.0001995*x*x -0.3834*x +317.6;
    }
    else{
        return -1;
    }
}
