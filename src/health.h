#ifndef HEALTH_H
#define HEALTH_H

#include <stdint.h>

// Self-checks: stack never used, where the main program stalled, why the last reset happened.

void health_init(void);         // first thing in main(): paints the free stack, reads the reset cause
void health_alive(void);        // main context, in every loop that waits
// The reason for the last reset: "power-on", "RESET (software)", "reset button"...
const char *health_reset_cause(void);
// Bytes of stack never used since boot (the stack meets the end of .bss).
uint32_t health_stack_free(void);
// 1 once after a stall longer than HEALTH_STALL_MS: how long and where (PC, LR).
uint8_t health_take_stall(uint32_t *ms, uint32_t *pc, uint32_t *lr);

#define HEALTH_STALL_MS 2000u

#endif // HEALTH_H
