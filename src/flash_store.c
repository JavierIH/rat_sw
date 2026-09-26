#include "flash_store.h"
#include <string.h>
#include "stm32f1xx_hal.h"
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

// ---- Freeze diagnostics (AGENTS.md, "Freezes") ----------------------------------
// Four freezes of ~200 s, all in flash writes that then failed, and a
// software reset that would not boot until a power cycle. The flash times its
// erase and program with the HSI oscillator (which the chip also boots on):
// each write checks it is running, starts it if not, and times itself with
// the cycle counter, which keeps counting while the CPU is stalled on the
// flash (SysTick does not: its handler is in the flash too). Anything odd
// is reported.
#define WRITE_SLOW_MS 200u

static uint32_t last_ms;

uint32_t flash_store_last_ms(void){
    return last_ms;
}

// The CPU stalls while the page is erased/programmed (~30 ms): only call this
// with the robot stopped.
uint8_t flash_store_write(const void *data, uint16_t len){
    if(len > FLASH_STORE_SIZE || (len & 3u)) return 0;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    const uint32_t rcc_cr = RCC->CR, sr_before = FLASH->SR, cr_before = FLASH->CR;
    const uint32_t cycles_per_ms = SystemCoreClock / 1000u;
    uint8_t hsi_started = 0;
    if(!(rcc_cr & RCC_CR_HSIRDY)){
        RCC->CR |= RCC_CR_HSION;
        const uint32_t t = DWT->CYCCNT;
        while(!(RCC->CR & RCC_CR_HSIRDY) && DWT->CYCCNT - t < 10u * cycles_per_ms){}
        hsi_started = 1;
    }
    const uint32_t tick0 = HAL_GetTick(), cycle0 = DWT->CYCCNT;
    HAL_FLASH_Unlock();
    uint8_t ok = erase_page();
    const uint8_t *src = (const uint8_t *)data;
    for(uint16_t i = 0; ok && i < len; i += 4){
        uint32_t word;
        memcpy(&word, src + i, sizeof(word));
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STORE_ADDR + i, word) == HAL_OK;
    }
    HAL_FLASH_Lock();
    ok = ok && memcmp((const void *)STORE_ADDR, data, len) == 0;
    last_ms = (DWT->CYCCNT - cycle0) / cycles_per_ms;     // wraps after ~60 s: see the ticks too
    const uint32_t ticks = HAL_GetTick() - tick0;
    if(!ok || hsi_started || last_ms > WRITE_SLOW_MS || ticks > WRITE_SLOW_MS
       || (sr_before & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR | FLASH_SR_BSY)) || (cr_before & ~FLASH_CR_LOCK)){
        print("!! flash: %s en %lu ms (%lu de SysTick)%s | antes RCC_CR=%08lx FLASH_SR=%02lx FLASH_CR=%04lx"
              " | despues RCC_CR=%08lx FLASH_SR=%02lx\n", ok ? "escrita" : "ERROR", (unsigned long)last_ms,
              (unsigned long)ticks, hsi_started ? " | HSI PARADO: arrancado" : "", (unsigned long)rcc_cr,
              (unsigned long)sr_before, (unsigned long)cr_before, (unsigned long)RCC->CR,
              (unsigned long)FLASH->SR);
    }
    return ok;
}
