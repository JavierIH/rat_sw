// RAM stand-in for the reserved flash page.
#include <string.h>
#include "flash_store.h"

unsigned char fake_flash[FLASH_STORE_SIZE];
int fake_flash_writes;

void fake_flash_wipe(void){
    memset(fake_flash, 0xFF, sizeof(fake_flash));   // erased flash reads all ones
}

const void *flash_store_data(void){
    return fake_flash;
}

uint8_t flash_store_write(const void *data, uint16_t len){
    if(len > FLASH_STORE_SIZE || (len & 3u)) return 0;
    fake_flash_wipe();
    memcpy(fake_flash, data, len);
    fake_flash_writes++;
    return 1;
}
