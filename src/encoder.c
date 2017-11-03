#include "encoder.h"


void ENCODER_Init(void){
    MX_TIM1_Init();
    MX_TIM2_Init();
    _encoder_state_l = 0;
    _encoder_state_r = 0;
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

void MX_TIM1_Init(void){
    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK){
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK){
        Error_Handler();
    }
}

/* TIM2 init function */
void MX_TIM2_Init(void){
    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_BOTHEDGE;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK){
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK){
        Error_Handler();
    }
}

uint16_t get_encoder(uint8_t encoder){
    if(encoder == ENCODER_L){
        return __HAL_TIM_GetCounter(&htim1);
    }
    else if(encoder == ENCODER_R){
        return __HAL_TIM_GetCounter(&htim2);
    }
    else{
        return 0;
    }
}

int32_t get_encoder_delta(uint8_t encoder){
    uint16_t encoder_ref = 0;
    if(encoder == ENCODER_L){
        encoder_ref = _encoder_state_l;
        _encoder_state_l = __HAL_TIM_GetCounter(&htim1);
        return _encoder_state_l - encoder_ref;
    }
    else if(encoder == ENCODER_R){
        encoder_ref = _encoder_state_r;
        _encoder_state_r = __HAL_TIM_GetCounter(&htim2);
        return _encoder_state_r  - encoder_ref;
    }
    else{
        return 0;
    }
}

int32_t get_encoder_diff(int32_t pos_a, int32_t pos_b){
     int32_t result = pos_b - pos_a;
     if(result > 32767){
         result -= 65535;
     }
     else if (result < -32767){
         result += 65535;
     }
     return result;
}
