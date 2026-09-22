#ifndef ENCODER_H
#define ENCODER_H

#include "error.h"

typedef enum {ENCODER_L, ENCODER_R} encoder_t;

extern TIM_HandleTypeDef htim1; //L
extern TIM_HandleTypeDef htim2; //R

extern uint16_t _encoder_state_r;
extern uint16_t _encoder_state_l;

void ENCODER_Init(void);
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void reset_encoder(encoder_t encoder);
uint16_t get_encoder(encoder_t encoder);
int32_t get_encoder_delta(encoder_t encoder);
int32_t get_encoder_diff(int32_t pos_a, int32_t pos_b);


#endif // ENCODER_H
