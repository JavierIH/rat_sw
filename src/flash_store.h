#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>

// One reserved flash page for persistent data. Implemented by flash_store.c
// on the robot and by a RAM fake in the host tests.
#define FLASH_STORE_SIZE 1024u

const void *flash_store_data(void);                         // FLASH_STORE_SIZE bytes, read-only
uint8_t flash_store_write(const void *data, uint16_t len);  // erase + program + verify; len multiple of 4

#endif // FLASH_STORE_H
