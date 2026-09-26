// RAM stand-in for the reserved flash pages, with the real rules: a program
// only fills erased space, an erase clears one page. It can wedge like the
// robot's flash (docs/freezes.md): after fake_flash_fail_after more halfwords
// the next one never lands and the store refuses everything until
// fake_flash_power_cycle().
#include <string.h>
#include "flash_store.h"

unsigned char fake_flash[FLASH_STORE_SIZE];
int fake_flash_programs, fake_flash_erases;
int fake_flash_fail_after = -1;     // halfwords (or erases) still fine; -1: never wedges
static uint8_t blocked;

void fake_flash_power_cycle(void){
    blocked = 0;
    fake_flash_fail_after = -1;
}

void fake_flash_wipe(void){
    memset(fake_flash, 0xFF, sizeof(fake_flash));   // erased flash reads all ones
    fake_flash_power_cycle();
}

const void *flash_store_data(void){
    return fake_flash;
}

uint8_t flash_store_blocked(void){
    return blocked;
}

uint8_t flash_store_erase(uint8_t page){
    if(blocked || page >= FLASH_STORE_PAGES) return 0;
    if(fake_flash_fail_after == 0){
        blocked = 1;
        return 0;
    }
    memset(fake_flash + page * FLASH_STORE_PAGE_SIZE, 0xFF, FLASH_STORE_PAGE_SIZE);
    fake_flash_erases++;
    return 1;
}

uint8_t flash_store_program(uint16_t offset, const void *data, uint16_t len){
    if(blocked || (offset & 1u) || (len & 1u) || offset + len > FLASH_STORE_SIZE) return 0;
    for(uint16_t i = 0; i < len; i++){
        if(fake_flash[offset + i] != 0xFFu) return 0;
    }
    fake_flash_programs++;
    for(uint16_t i = 0; i < len; i += 2u){
        if(fake_flash_fail_after == 0){
            blocked = 1;
            return 0;
        }
        if(fake_flash_fail_after > 0) fake_flash_fail_after--;
        memcpy(fake_flash + offset + i, (const unsigned char *)data + i, 2);
    }
    return 1;
}
