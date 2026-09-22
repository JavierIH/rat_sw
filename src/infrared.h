#ifndef INFRARED_H
#define INFRARED_H

#include "stm32f1xx_hal.h"

typedef enum {IR_FL, IR_FR, IR_SL, IR_SR} ir_sensor_t;

extern volatile uint32_t _adc_buf[4];
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;

void IR_Init();
int get_ir(ir_sensor_t ir);
float get_ir_mm(ir_sensor_t ir);


#endif // INFRARED_H
