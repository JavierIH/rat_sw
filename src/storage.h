#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>

// Map, goal and parameters in flash: a log of records in two pages, programmed during runs, erased only at boot.

typedef enum {
    STORAGE_LOADED,         // map, goal and parameters restored
    STORAGE_NEW_DEFAULTS,   // map and goal restored; the firmware's parameter defaults changed, so they stay
    STORAGE_EMPTY,          // nothing saved
    STORAGE_CORRUPT,        // bad checksum/format: ignored
    STORAGE_STALE,          // another record layout, or firmware for another maze (goal): ignored
} storage_status_t;

typedef enum {
    STORAGE_FAILED,         // the flash failed or was slow: blocked until a power cycle, the map stays in RAM
    STORAGE_WRITTEN,
    STORAGE_UNCHANGED,      // the current record already holds exactly this: nothing written
    STORAGE_FULL,           // no erased slot left until the next boot compacts (or SAVE)
} storage_save_t;

// Current map, goal and parameters; map_only (run ends): nothing written if only the parameters changed.
storage_save_t storage_save(uint8_t map_only);
storage_status_t storage_load(void);  // restores what the status says, nothing otherwise
// Boot (or SAVE when full): frees every slot but the current record's (at most two erases); healthy flash only.
uint8_t storage_needs_compact(void);
uint8_t storage_compact(void);
uint8_t storage_free_slots(void);
const char *storage_status_name(storage_status_t status);

#endif // STORAGE_H
