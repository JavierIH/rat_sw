#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>

// One reserved flash page for persistent data. Implemented by flash_store.c
// on the robot and by a RAM fake in the host tests.
#define FLASH_STORE_SIZE 1024u

const void *flash_store_data(void);                         // FLASH_STORE_SIZE bytes, read-only
uint8_t flash_store_write(const void *data, uint16_t len);  // erase + program + verify; len multiple of 4

// Robot only (flash_store.c): the protection against a wedged flash
// (docs/freezes.md) and what the last write took.
typedef struct {
    uint32_t probe_us;      // one spare halfword programmed before the erase (0: no room for it)
    uint32_t erase_ms, program_ms;
} flash_timing_t;

const flash_timing_t *flash_store_timing(void);
uint8_t flash_store_blocked(void);          // a write failed or was slow: none until a power cycle
void flash_store_settle(uint32_t ms);       // motors off this long before writing (CAL FLASH: 0)

#endif // FLASH_STORE_H
