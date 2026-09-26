#include "health.h"
#include "stm32f1xx_hal.h"
#include "uart.h"

#define STACK_PAINT 0x5AA5C33Cu

extern uint32_t _ebss;      // linker: end of .bss, where the free RAM (and the stack's reach) begins

static const char *reset_cause = "?";
static volatile uint32_t alive_ms;
static volatile uint8_t stalled, stall_ready;
static volatile uint32_t stall_start, stall_ms, stall_pc, stall_lr;

#define RCC_WATCH (RCC_CR_HSION | RCC_CR_HSIRDY | RCC_CR_HSEON | RCC_CR_HSERDY | RCC_CR_PLLON | RCC_CR_PLLRDY \
                   | RCC_CR_CSSON)
static uint32_t rcc_expected;       // 0 until the clock setup is done
static uint32_t rcc_seen;
static uint8_t rcc_changed;

// Every word between the end of .bss and a little below the stack pointer
// gets a pattern; whatever is still the pattern later was never used.
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
                : (csr & RCC_CSR_PORRSTF) ? "encendido"
                : (csr & RCC_CSR_LPWRRSTF) ? "bajo consumo"
                : (csr & RCC_CSR_WWDGRSTF) ? "watchdog de ventana"
                : (csr & RCC_CSR_PINRSTF) ? "boton de reset"
                : "?";
    RCC->CSR |= RCC_CSR_RMVF;
    alive_ms = HAL_GetTick();
}

void health_alive(void){
    const uint32_t now = HAL_GetTick();
    alive_ms = now;
    const uint32_t cr = RCC->CR & RCC_WATCH;
    if(rcc_expected && cr != rcc_expected && !rcc_changed){
        rcc_seen = cr;
        rcc_changed = 1;
        if(!(cr & RCC_CR_HSION)) RCC->CR |= RCC_CR_HSION;
    }
    if(stalled){
        stall_ms = now - stall_start;
        stalled = 0;
        stall_ready = 1;
    }
}

void health_clock_baseline(void){
    rcc_expected = RCC->CR & RCC_WATCH;
    rcc_changed = 0;
}

uint8_t health_take_clock_change(uint32_t *expected, uint32_t *seen){
    if(!rcc_changed) return 0;
    *expected = rcc_expected;
    *seen = rcc_seen;
    health_clock_baseline();    // report each change once
    return 1;
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

// SysTick, with the frame the interrupt stacked: if the main program has not
// said it is alive for a while, note where it is (frame[6] = PC, [5] = LR).
void app_stall_check(const uint32_t *frame){
    if(!stalled && !stall_ready && HAL_GetTick() - alive_ms > HEALTH_STALL_MS){
        stalled = 1;
        stall_start = alive_ms;
        stall_pc = frame[6];
        stall_lr = frame[5];
    }
}
