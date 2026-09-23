#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// Quadrature encoders: TIM1 = left, TIM2 = right (x4 counting).

typedef enum { ENCODER_L, ENCODER_R } encoder_t;

void ENCODER_Init(void);
void encoder_tick(void);                    // every 1 ms (SysTick): extends the counters to 32 bits
int32_t encoder_total(encoder_t encoder);   // ticks since boot, forward positive, no wrap-around
uint16_t encoder_raw(encoder_t encoder);    // hardware counter, left inverted so forward counts up
uint32_t encoder_idle_ms(void);             // time since either wheel last moved

#endif // ENCODER_H
