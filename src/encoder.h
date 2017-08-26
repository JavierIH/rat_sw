#ifndef ENCODER_H
#define ENCODER_H

#include "error.h"

#define ENCODER_L               0
#define ENCODER_R               1

TIM_HandleTypeDef htim1; //L
TIM_HandleTypeDef htim2; //R

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
int32_t get_encoder(uint8_t encoder);


#endif // ENCODER_H
