#ifndef INFRARED_H
#define INFRARED_H

#include <stdint.h>

// Four IR distance sensors on ADC1, sampled continuously by circular DMA with
// no CPU involvement. Each reading averages the last IR_OVERSAMPLE conversions.

#define IR_OVERSAMPLE 16

typedef enum { IR_FL, IR_FR, IR_SL, IR_SR, IR_COUNT } ir_sensor_t;

void IR_Init(void);
uint16_t ir_raw(ir_sensor_t ir);    // averaged ADC counts, 0..4095
float ir_mm(ir_sensor_t ir);        // calibrated distance, clamped to 0..400 mm
const char *ir_calibration_text(ir_sensor_t ir);   // "a, b, c, d" exactly as compiled

#endif // INFRARED_H
