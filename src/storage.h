#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>

// Persistence of the maze map, goal and runtime parameters in flash.

typedef enum {
    STORAGE_LOADED,         // map, goal and parameters restored
    STORAGE_NEW_DEFAULTS,   // map and goal restored; the firmware's parameter defaults changed, so they stay
    STORAGE_EMPTY,          // nothing saved
    STORAGE_CORRUPT,        // bad checksum/format: ignored
    STORAGE_STALE,          // another record layout, or firmware for another maze (goal): ignored
} storage_status_t;

// Current map, goal and parameters: 1 written, STORAGE_UNCHANGED if the
// flash already held exactly them (nothing written), 0 on failure.
#define STORAGE_UNCHANGED 2u
uint8_t storage_save(void);
uint8_t storage_rewrite(void);        // the same, written even if unchanged (CAL FLASH)
storage_status_t storage_load(void);  // restores what the status says, nothing otherwise
const char *storage_status_name(storage_status_t status);

#endif // STORAGE_H
