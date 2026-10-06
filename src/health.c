#include "health.h"
#include "stm32f1xx_hal.h"
#include "uart.h"

#define STACK_PAINT 0x5AA5C33Cu

extern uint32_t _ebss;      // linker: end of .bss, where the free RAM (and the stack's reach) begins

static const char *reset_cause = "?";
static volatile uint32_t alive_ms;
static volatile uint8_t stalled, stall_ready;
static volatile uint32_t stall_start, stall_ms, stall_pc, stall_lr;
static volatile uint32_t tick_cycles_max;

// A pattern from the end of .bss to below the stack pointer; whatever still holds it was never used.
static void paint_stack(void){
    uint32_t *p = &_ebss;
    uint32_t *const top = (uint32_t *)(__get_MSP() - 64u);
    while(p < top) *p++ = STACK_PAINT;
}

void health_init(void){
    paint_stack();
    const uint32_t csr = RCC->CSR;
    reset_cause = (csr & RCC_CSR_IWDGRSTF) ? "watchdog"
                : (csr & RCC_CSR_SFTRSTF) ? "RESET (software)"
                : (csr & RCC_CSR_PORRSTF) ? "power-on"
                : (csr & RCC_CSR_LPWRRSTF) ? "low power"
                : (csr & RCC_CSR_WWDGRSTF) ? "window watchdog"
                : (csr & RCC_CSR_PINRSTF) ? "reset button"
                : "?";
    RCC->CSR |= RCC_CSR_RMVF;
    alive_ms = HAL_GetTick();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     // the cycle counter: SysTick and flash timings
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void app_tick_time(uint32_t cycles){
    if(cycles > tick_cycles_max) tick_cycles_max = cycles;
}

uint32_t health_tick_us_max(void){
    return tick_cycles_max / (SystemCoreClock / 1000000u);
}

void health_alive(void){
    const uint32_t now = HAL_GetTick();
    alive_ms = now;
    if(stalled){
        stall_ms = now - stall_start;
        stalled = 0;
        stall_ready = 1;
    }
}

// uart.c: waiting for room to print.
void uart_waiting(void){
    health_alive();
}

const char *health_reset_cause(void){
    return reset_cause;
}

uint32_t health_stack_free(void){
    const uint32_t *p = &_ebss;
    uint32_t words = 0;
    while(p[words] == STACK_PAINT) words++;
    return words * 4u;
}

uint8_t health_take_stall(uint32_t *ms, uint32_t *pc, uint32_t *lr){
    if(!stall_ready) return 0;
    stall_ready = 0;
    *ms = stall_ms;
    *pc = stall_pc;
    *lr = stall_lr;
    return 1;
}

// SysTick: if the main program has not said it is alive for a while, note where it is (frame[6] PC, [5] LR).
void app_stall_check(const uint32_t *frame){
    if(!stalled && !stall_ready && HAL_GetTick() - alive_ms > HEALTH_STALL_MS){
        stalled = 1;
        stall_start = alive_ms;
        stall_pc = frame[6];
        stall_lr = frame[5];
    }
}
