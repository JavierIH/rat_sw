#include "flash_store.h"
#include <string.h>
#include "stm32f1xx_hal.h"
#include "motor.h"
#include "robot_config.h"
#include "uart.h"

// The last two 1 KB pages of the 64 KB part. platformio.ini caps the program
// at 62 KB (board_upload.maximum_size), so the build fails before code could
// reach them.
#define STORE_ADDR 0x0800F800u

_Static_assert(FLASH_STORE_PAGE_SIZE == FLASH_PAGE_SIZE, "a store page must be exactly one flash page");

const void *flash_store_data(void){
    return (const void *)STORE_ADDR;
}

// ---- Protection against a wedged flash (docs/freezes.md) ------------------------------
// This chip (a clone: DBGMCU_IDCODE 0x307) sometimes wedges its flash until a
// power cycle: every operation then takes ~9000 times longer (an erase ~200 s
// instead of 22 ms, a halfword ~0.45 s instead of 56 us), and as the CPU runs
// from the flash it stalls meanwhile. An erase cannot be cut short, so
// storage.c erases only at boot; runs only program erased space, a halfword at
// a time, each timed (one slow halfword costs well under a second, not
// minutes). The flash times its program and erase with the HSI, whatever
// clock runs the CPU, and restarting the HSI cleared a wedge (09-27): so the
// first slow halfword restarts it and the write goes on; a second slow one
// (the restart did not help) or a failed one stops the write, and the store
// refuses everything after it until a power cycle. The cycle counter times
// them: it keeps counting while the CPU is stalled on the flash (SysTick
// does not).
#define HALFWORD_SLOW_US    1000u   // normal: ~56 us
#define ERASE_SLOW_MS       200u    // normal: ~22 ms
#define SETTLE_EXTRA_MS     2000u   // waiting for the UART: at most this beyond FLASH_SETTLE_MS
#define HALFWORD_FAILED     0xFFFFFFFFu

static flash_timing_t timing;
static uint8_t blocked;

const flash_timing_t *flash_store_timing(void){
    return &timing;
}

uint8_t flash_store_blocked(void){
    return blocked;
}

static void cycle_counter_on(void){
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t cycles_per_us(void){
    return SystemCoreClock / 1000000u;
}

// Every wedged write began within ms of the end of a move; writes made with
// the motors off a while and the UART quiet never wedged (92 of 92). Motors
// off FLASH_SETTLE_MS and nothing left to send, bounded.
static void settle(void){
    const uint32_t t0 = HAL_GetTick();
    while((motor_idle_ms() < FLASH_SETTLE_MS || !uart_tx_idle()) && HAL_GetTick() - t0 < FLASH_SETTLE_MS + SETTLE_EXTRA_MS){
        uart_waiting();
    }
}

// The flash times its erase and program with the HSI: it must run. Bounded by
// iterations too, in case the cycle counter did not start.
static void hsi_wait(uint32_t ready){
    const uint32_t t = DWT->CYCCNT, limit = 10000u * cycles_per_us();
    for(uint32_t n = 0; ((RCC->CR & RCC_CR_HSIRDY) != 0) != ready && DWT->CYCCNT - t < limit && n < 200000u; n++){}
}

static uint8_t hsi_on(void){
    if(RCC->CR & RCC_CR_HSIRDY) return 1;
    RCC->CR |= RCC_CR_HSION;
    hsi_wait(1);
    print("!! flash: el HSI estaba parado: arrancado (%s)\n", (RCC->CR & RCC_CR_HSIRDY) ? "listo" : "NO arranca");
    return (RCC->CR & RCC_CR_HSIRDY) != 0;
}

static void report(const char *what, uint32_t hal_error, uint32_t rcc_cr, uint32_t acr){
    blocked = 1;
    print("!! flash: %s | 1a %lu us, peor %lu us, total %lu ms, borrado %lu ms | HAL %lx\n", what,
          (unsigned long)timing.first_us, (unsigned long)timing.worst_us, (unsigned long)timing.program_ms,
          (unsigned long)timing.erase_ms, (unsigned long)hal_error);
    print("!! flash: antes RCC_CR=%08lx ACR=%02lx | despues RCC_CR=%08lx SR=%02lx CR=%04lx\n", (unsigned long)rcc_cr,
          (unsigned long)acr, (unsigned long)RCC->CR, (unsigned long)FLASH->SR, (unsigned long)FLASH->CR);
    print("!! flash bloqueada hasta apagar y encender: el mapa sigue en RAM (no uses RESET)\n");
}

uint8_t flash_store_erase(uint8_t page){
    if(blocked || page >= FLASH_STORE_PAGES) return 0;
    settle();
    cycle_counter_on();
    const uint32_t rcc_cr = RCC->CR, acr = FLASH->ACR;
    if(!hsi_on()) return 0;
    const uint32_t tick0 = HAL_GetTick(), t0 = DWT->CYCCNT;
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase;
    memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.PageAddress = STORE_ADDR + page * FLASH_STORE_PAGE_SIZE;
    erase.NbPages = 1;
    uint32_t page_error = 0;
    uint8_t ok = HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK;
    const uint32_t hal_error = HAL_FLASH_GetError();
    HAL_FLASH_Lock();
    timing.erase_ms = (DWT->CYCCNT - t0) / (cycles_per_us() * 1000u);   // wraps after ~60 s: see the ticks too
    const volatile uint32_t *w = (const volatile uint32_t *)erase.PageAddress;
    for(uint16_t i = 0; ok && i < FLASH_STORE_PAGE_SIZE / 4u; i++) ok = w[i] == 0xFFFFFFFFu;
    if(!ok || timing.erase_ms > ERASE_SLOW_MS || HAL_GetTick() - tick0 > ERASE_SLOW_MS){
        report(ok ? "borrado lento" : "borrado fallido", hal_error, rcc_cr, acr);
        return 0;
    }
    return 1;
}

// Programs one halfword and times it (us); HALFWORD_FAILED if it failed.
static uint32_t program_halfword(uint32_t address, uint16_t value){
    const uint32_t t = DWT->CYCCNT;
    const uint8_t ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, value) == HAL_OK;
    const uint32_t us = (DWT->CYCCNT - t) / cycles_per_us();
    return ok ? us : HALFWORD_FAILED;
}

// Off and on again (a few us). Not while the HSI runs the CPU (the crystal
// failed): the RCC ignores clearing HSION then.
static uint8_t hsi_restart(void){
    const uint32_t cfgr = RCC->CFGR, sws = cfgr & RCC_CFGR_SWS;
    if(sws == RCC_CFGR_SWS_HSI || (sws == RCC_CFGR_SWS_PLL && !(cfgr & RCC_CFGR_PLLSRC))) return 0;
    RCC->CR &= ~RCC_CR_HSION;
    hsi_wait(0);
    RCC->CR |= RCC_CR_HSION;
    hsi_wait(1);
    timing.hsi_restarts++;
    return (RCC->CR & RCC_CR_HSIRDY) != 0;
}

uint8_t flash_store_program(uint16_t offset, const void *data, uint16_t len){
    if(blocked || (offset & 1u) || (len & 1u) || (uint32_t)offset + len > FLASH_STORE_SIZE) return 0;
    const volatile uint16_t *dst = (const volatile uint16_t *)(STORE_ADDR + offset);
    for(uint16_t i = 0; i < len / 2u; i++){
        if(dst[i] != 0xFFFFu) return 0;     // not erased: the caller's mistake, not the flash's
    }
    settle();
    cycle_counter_on();
    const uint32_t rcc_cr = RCC->CR, acr = FLASH->ACR;
    if(!hsi_on()) return 0;
    const uint32_t t0 = DWT->CYCCNT;
    timing.first_us = timing.worst_us = 0;
    HAL_FLASH_Unlock();
    const uint8_t *src = (const uint8_t *)data;
    uint8_t ok = 1, restarted = 0, slow_right = 0;
    uint32_t slow_us = 0, after_us = 0;    // the slow halfword; the slowest after the restart
    for(uint16_t i = 0; ok && i < len; i += 2u){
        uint16_t value;
        memcpy(&value, src + i, sizeof(value));
        const uint32_t us = program_halfword(STORE_ADDR + offset + i, value);
        if(i == 0) timing.first_us = us;
        if(us > timing.worst_us) timing.worst_us = us;
        if(restarted && us > after_us) after_us = us;
        if(us <= HALFWORD_SLOW_US) continue;
        // Slow: restart the HSI once and go on; the next halfword tells.
        ok = us != HALFWORD_FAILED && !restarted && hsi_restart();
        if(ok){
            restarted = 1;
            slow_us = us;
            slow_right = dst[i / 2u] == value;
        }
    }
    const uint32_t hal_error = HAL_FLASH_GetError();
    HAL_FLASH_Lock();
    timing.program_ms = (DWT->CYCCNT - t0) / (cycles_per_us() * 1000u);
    const uint8_t right = memcmp((const void *)dst, data, len) == 0;
    if(restarted){
        print("!! flash: 2 bytes en %lu us (dato %s): HSI reiniciado; despues, peor %lu us (normal ~56)\n",
              (unsigned long)slow_us, slow_right ? "bien" : "MAL", (unsigned long)after_us);
    }
    if(ok && right) return 1;
    // The slow halfword landed wrong but the restart cured the flash: the
    // store goes on (storage.c tries the next slot).
    if(ok && restarted) return 0;
    report(timing.worst_us == HALFWORD_FAILED ? "escritura fallida" : ok ? "verificacion fallida"
           : "escritura LENTA, cortada", hal_error, rcc_cr, acr);
    return 0;
}

uint8_t flash_store_probe(void){
    if(blocked) return 0;
    cycle_counter_on();
    const uint32_t rcc_cr = RCC->CR, acr = FLASH->ACR;
    uint8_t restarted = 0;
    for(uint8_t page = 0; page < FLASH_STORE_PAGES; page++){
        const uint32_t base = STORE_ADDR + (page + 1u) * FLASH_STORE_PAGE_SIZE - FLASH_STORE_SPARE;
        for(uint32_t a = base; a < base + FLASH_STORE_SPARE; a += 2u){
            if(*(const volatile uint16_t *)a != 0xFFFFu) continue;
            if(!hsi_on()) return 0;
            HAL_FLASH_Unlock();
            const uint32_t us = program_halfword(a, 0u);
            const uint32_t hal_error = HAL_FLASH_GetError();
            HAL_FLASH_Lock();
            timing.first_us = timing.worst_us = us;
            if(us <= HALFWORD_SLOW_US) return 1;
            if(us == HALFWORD_FAILED || restarted || !hsi_restart()){
                report("prueba LENTA", hal_error, rcc_cr, acr);
                return 0;
            }
            restarted = 1;
            print("!! flash: prueba lenta (%lu us): HSI reiniciado, otra\n", (unsigned long)us);
        }
    }
    return 0;   // no spare halfword left: cannot tell
}
