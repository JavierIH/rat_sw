#ifndef INFRARED_H
#define INFRARED_H

#include "stm32f1xx_hal.h"

typedef enum {IR_FL, IR_FR, IR_SL, IR_SR} ir_sensor_t;

ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef _sConfig;

void IR_Init();
int get_ir(ir_sensor_t ir);


#endif // INFRARED_H
