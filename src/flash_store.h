#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>

// Two reserved flash pages for persistent data (flash_store.c on the robot, a RAM fake in the host tests).
#define FLASH_STORE_PAGES       2u
#define FLASH_STORE_PAGE_SIZE   1024u
#define FLASH_STORE_SIZE        (FLASH_STORE_PAGES * FLASH_STORE_PAGE_SIZE)

const void *flash_store_data(void);     // FLASH_STORE_SIZE bytes, read-only; erased flash reads 0xFF
uint8_t flash_store_erase(uint8_t page);
// Programs `len` bytes at an erased `offset`; a slow halfword restarts the HSI once; a second one blocks the store.
uint8_t flash_store_program(uint16_t offset, const void *data, uint16_t len);
uint8_t flash_store_blocked(void);
#define FLASH_STORE_SPARE       4u      // bytes at the end of each page that records leave for the probe

// Robot only: the last operations' times, and the check before erasing at a boot that was not a power-on.
typedef struct {
    uint32_t first_us, worst_us;    // the last program: its first halfword and its slowest one
    uint32_t program_ms, erase_ms;  // the last program and the last erase, whole
    uint32_t hsi_restarts;          // slow halfwords met by restarting the HSI again, since the boot
} flash_timing_t;

const flash_timing_t *flash_store_timing(void);
uint8_t flash_store_probe(void);        // one spare halfword programmed and timed: 1 if normal

#endif // FLASH_STORE_H
