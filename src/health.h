#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>

// Self-checks: stack never used, where the main program stalled, why the last reset happened.

void health_init(void);         // first thing in main(): paints the free stack, reads the reset cause
void health_alive(void);        // main context, in every loop that waits
// The reason for the last reset: "encendido", "RESET", "boton"...
const char *health_reset_cause(void);
// 1 if the last reset was a power-on (or brown-out): the flash is healthy then.
uint8_t health_power_on(void);
// Bytes of stack never used since boot (the stack meets the end of .bss).
uint32_t health_stack_free(void);
// 1 once after a stall longer than HEALTH_STALL_MS: how long and where (PC, LR).
uint8_t health_take_stall(uint32_t *ms, uint32_t *pc, uint32_t *lr);

// The clock bits of RCC->CR as the clock setup left them; health_alive() keeps the first unrequested change.
void health_clock_baseline(void);
uint8_t health_take_clock_change(uint32_t *expected, uint32_t *seen);

#define HEALTH_STALL_MS 2000u

#endif // HEALTH_H
