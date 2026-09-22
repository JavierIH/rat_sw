#include "flash_store.h"
#include <string.h>
#include "stm32f1xx_hal.h"

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

// The CPU stalls while the page is erased/programmed (~30 ms): only call this
// with the robot stopped.
uint8_t flash_store_write(const void *data, uint16_t len){
    if(len > FLASH_STORE_SIZE || (len & 3u)) return 0;
    HAL_FLASH_Unlock();
    uint8_t ok = erase_page();
    const uint8_t *src = (const uint8_t *)data;
    for(uint16_t i = 0; ok && i < len; i += 4){
        uint32_t word;
        memcpy(&word, src + i, sizeof(word));
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, STORE_ADDR + i, word) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok && memcmp((const void *)STORE_ADDR, data, len) == 0;
}
