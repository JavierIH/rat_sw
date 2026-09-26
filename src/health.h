#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>

// Self-checks to catch rare failures on the robot and say what they were:
// how much stack is ever used, where the main program was if it stopped
// responding, why the last reset happened.

void health_init(void);         // first thing in main(): paints the free stack, reads the reset cause
void health_alive(void);        // main context, in every loop that waits
// The reason for the last reset: "encendido", "RESET", "boton"...
const char *health_reset_cause(void);
// 1 if the last reset was a power-on (or brown-out): the flash is healthy
// then (docs/freezes.md: a power cycle always cured a wedged flash).
uint8_t health_power_on(void);
// Bytes of stack never used since boot (the stack meets the end of .bss).
uint32_t health_stack_free(void);
// 1 once after the main program stopped responding for more than
// HEALTH_STALL_MS: how long, and where it was (program counter and return
// address when it was caught).
uint8_t health_take_stall(uint32_t *ms, uint32_t *pc, uint32_t *lr);

// The oscillators in RCC->CR as the clock setup left them (call after every
// intended clock change). health_alive() watches them: the freezes all came
// in flash writes, and the flash needs the HSI on to erase and program. A
// change nobody asked for turns the HSI back on and is kept (the first one)
// for the main loop to report.
void health_clock_baseline(void);
uint8_t health_take_clock_change(uint32_t *expected, uint32_t *seen);

#define HEALTH_STALL_MS 2000u

#endif // HEALTH_H
