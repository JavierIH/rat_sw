#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>

// Two reserved flash pages at the end of the flash for persistent data.
// Implemented by flash_store.c on the robot and by a RAM fake in the host
// tests. storage.c keeps a log of records in them: during runs it only
// programs erased space, a halfword at a time, and it erases pages only at
// boot (docs/freezes.md: an erase on a wedged flash stalls the CPU ~200 s,
// a halfword ~0.5 s).
#define FLASH_STORE_PAGES       2u
#define FLASH_STORE_PAGE_SIZE   1024u
#define FLASH_STORE_SIZE        (FLASH_STORE_PAGES * FLASH_STORE_PAGE_SIZE)

const void *flash_store_data(void);     // FLASH_STORE_SIZE bytes, read-only; erased flash reads 0xFF
uint8_t flash_store_erase(uint8_t page);
// Programs `len` bytes (even) at `offset`, which must read erased. Stops at
// the first halfword that fails or is slow; then, and after a failed or slow
// erase, the store refuses everything until a power cycle.
uint8_t flash_store_program(uint16_t offset, const void *data, uint16_t len);
uint8_t flash_store_blocked(void);
#define FLASH_STORE_SPARE       4u      // bytes at the end of each page that records leave for the probe

// Robot only (flash_store.c): what the last operations took, and the check
// before erasing at a boot that was not a power-on (the flash may still be
// wedged then; a power cycle always cured it).
typedef struct {
    uint32_t first_us, worst_us;    // the last program: its first halfword and its slowest one
    uint32_t program_ms, erase_ms;  // the last program and the last erase, whole
} flash_timing_t;

const flash_timing_t *flash_store_timing(void);
uint8_t flash_store_probe(void);        // one spare halfword programmed and timed: 1 if normal

#endif // FLASH_STORE_H
