#include "flash_store.h"
#include <string.h>
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "robot_config.h"
#include "uart.h"

// Last 1 KB page of the 64 KB part. platformio.ini caps the program at 63 KB
// (board_upload.maximum_size), so the build fails before code could reach it.
#define STORE_ADDR 0x0800FC00u

_Static_assert(FLASH_STORE_SIZE == FLASH_PAGE_SIZE, "the store must be exactly one flash page");

const void *flash_store_data(void){
    return (const void *)STORE_ADDR;
}

static uint8_t erase_page(void){
    FLASH_EraseInitTypeDef erase;
    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.PageAddress = STORE_ADDR;
    erase.NbPages = 1;
    uint32_t page_error = 0;
    return HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK;
}

// ---- Protection against a wedged flash (docs/freezes.md) ------------------------------
// Some writes wedge the flash until a power cycle: every later erase stalls
// the CPU (it runs from the flash) ~200 s and fails, and programming one
// halfword takes ~20 ms instead of ~50 us. Every such write began within ms
// of the end of a move. So a write:
// - waits until the motors have been off FLASH_SETTLE_MS and the UART is idle;
// - first programs one spare halfword after the record and times it: slow
//   or failed means wedged, and then it does not erase (the saved record
//   survives, the stall is ~20 ms instead of ~200 s);
// - after any failed or slow write refuses every later one until a power
//   cycle (a software reset may not even boot then).
// The cycle counter times it: it keeps counting while the CPU is stalled on
// the flash (SysTick does not: its handler is in the flash too).
#define WRITE_SLOW_MS       200u    // a page erase and ~150 words: ~38 ms
#define PROBE_SLOW_US       1000u   // one halfword: ~50 us; wedged: ~20 ms
#define SETTLE_EXTRA_MS     2000u   // waiting for the UART: at most this beyond settle_ms

static uint32_t settle_ms = FLASH_SETTLE_MS;
static flash_timing_t timing;
static uint8_t blocked;

const flash_timing_t *flash_store_timing(void){
    return &timing;
}

uint8_t flash_store_blocked(void){
    return blocked;
}

void flash_store_settle(uint32_t ms){
    settle_ms = ms;
}

// Motors off for settle_ms and nothing left to send (bounded). 0: no wait
// at all, as before the protection (CAL FLASH).
static void settle(void){
    if(!settle_ms) return;
    const uint32_t t0 = HAL_GetTick();
    while((motor_idle_ms() < settle_ms || !uart_tx_idle()) && HAL_GetTick() - t0 < settle_ms + SETTLE_EXTRA_MS){
        uart_waiting();
    }
}

// First erased halfword after the record: every write erases the page, so
// there is room unless the page holds something else there.
static uint32_t probe_address(uint16_t len){
    for(uint32_t a = STORE_ADDR + len; a < STORE_ADDR + FLASH_STORE_SIZE; a += 2u){
        if(*(const volatile uint16_t *)a == 0xFFFFu) return a;
    }
    return 0;
}

// The CPU stalls while the page is erased/programmed (~38 ms): only call this
// with the robot stopped.
uint8_t flash_store_write(const void *data, uint16_t len){
    if(len > FLASH_STORE_SIZE || (len & 3u)) return 0;
    if(blocked){
        print("!! flash bloqueada por un fallo anterior: no escribo hasta apagar y encender\n");
        return 0;
    }
    settle();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t rcc_cr = RCC->CR, sr_before = FLASH->SR, cr_before = FLASH->CR, acr = FLASH->ACR;
    const uint32_t cycles_per_us = SystemCoreClock / 1000000u, cycles_per_ms = cycles_per_us * 1000u;
    uint8_t hsi_started = 0;
    if(!(rcc_cr & RCC_CR_HSIRDY)){
        // The flash times its erase and program with the HSI.
        RCC->CR |= RCC_CR_HSION;
        // Bounded by iterations too: a clone chip may lack the cycle counter.
        const uint32_t t = DWT->CYCCNT;
        for(uint32_t n = 0; !(RCC->CR & RCC_CR_HSIRDY) && DWT->CYCCNT - t < 10u * cycles_per_ms && n < 200000u; n++){}
        hsi_started = 1;
    }
    memset(&timing, 0, sizeof(timing));
    const uint32_t tick0 = HAL_GetTick();
    HAL_FLASH_Unlock();
    uint8_t ok = 1;
    const uint32_t probe = probe_address(len);
    uint32_t t = DWT->CYCCNT;
    if(probe){
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, probe, 0u) == HAL_OK;
        timing.probe_us = (DWT->CYCCNT - t) / cycles_per_us;
    }
    const uint8_t wedged = !ok || timing.probe_us > PROBE_SLOW_US;
    if(!wedged){
        t = DWT->CYCCNT;
        ok = erase_page();
        timing.erase_ms = (DWT->CYCCNT - t) / cycles_per_ms;
        t = DWT->CYCCNT;
        const uint8_t *src = (const uint8_t *)data;
        for(uint16_t i = 0; ok && i < len; i += 4){
            uint32_t word;
            memcpy(&word, src + i, sizeof(word));
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STORE_ADDR + i, word) == HAL_OK;
        }
        timing.program_ms = (DWT->CYCCNT - t) / cycles_per_ms;   // wraps after ~60 s: see the ticks too
    }
    const uint32_t hal_error = HAL_FLASH_GetError();
    HAL_FLASH_Lock();
    ok = ok && !wedged && memcmp((const void *)STORE_ADDR, data, len) == 0;
    const uint32_t ticks = HAL_GetTick() - tick0;
    const uint8_t slow = timing.erase_ms + timing.program_ms > WRITE_SLOW_MS || ticks > WRITE_SLOW_MS;
    if(!ok || slow) blocked = 1;
    if(blocked || hsi_started || (sr_before & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_BSY))
       || (cr_before & ~FLASH_CR_LOCK)){
        print("!! flash: %s | prueba %lu us, borrado %lu ms, escritura %lu ms, tick %lu ms | HAL %lx\n",
              wedged ? "ATASCADA, no borro" : !ok ? "ERROR" : slow ? "lenta" : "aviso",
              (unsigned long)timing.probe_us, (unsigned long)timing.erase_ms, (unsigned long)timing.program_ms,
              (unsigned long)ticks, (unsigned long)hal_error);
        print("!! flash antes: RCC_CR=%08lx SR=%02lx CR=%04lx ACR=%02lx%s | despues: RCC_CR=%08lx SR=%02lx\n",
              (unsigned long)rcc_cr, (unsigned long)sr_before, (unsigned long)cr_before, (unsigned long)acr,
              hsi_started ? " HSI PARADO: arrancado" : "", (unsigned long)RCC->CR, (unsigned long)FLASH->SR);
        if(blocked) print("!! flash bloqueada: no escribo mas hasta apagar y encender (un RESET puede no arrancar)\n");
    }
    return ok;
}
