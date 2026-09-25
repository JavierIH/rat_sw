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

uint8_t storage_save(void);           // current map, goal and parameters
storage_status_t storage_load(void);  // restores what the status says, nothing otherwise
const char *storage_status_name(storage_status_t status);

#endif // STORAGE_H
