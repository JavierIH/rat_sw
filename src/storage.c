#include "storage.h"
#include <stddef.h>
#include <string.h>
#include "crc32.h"
#include "flash_store.h"
#include "maze.h"
#include "params.h"
#include "robot_config.h"

#define STORE_MAGIC     0x4D544152u     // "RATM"
// 4: speeds in mm/s, deg/s; 5: curve speed; 6: two signatures; 7: a log with sequence numbers.
#define STORE_VERSION   7u

// A log of records (docs/freezes.md): saves program an erased slot, the highest sequence wins; only the boot erases.
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t seq;               // one more than the record before it
    uint32_t maze_signature;    // maze_defaults_signature() of the firmware that saved it
    uint32_t params_signature;  // params_defaults_signature() of the firmware that saved it
    maze_snapshot_t maze;
    params_t params;
    uint32_t crc;           // CRC-32 of every byte before this field
} record_t;

#define SLOTS_PER_PAGE  3u
#define SLOTS           (SLOTS_PER_PAGE * FLASH_STORE_PAGES)

_Static_assert(sizeof(record_t) % 4 == 0, "records must keep their fields aligned in flash");
_Static_assert(SLOTS_PER_PAGE * sizeof(record_t) <= FLASH_STORE_PAGE_SIZE - FLASH_STORE_SPARE,
               "three records must fit in a page");
_Static_assert(offsetof(record_t, crc) == sizeof(record_t) - 4, "crc must be the last field");

static record_t record;     // static: too big for the stack budget
static int8_t current = -1; // slot of the current record, -1 if none
static uint32_t current_seq;

// A firmware built for another maze (default goal, virtual robot) must not load this map.
static uint32_t maze_defaults_signature(void){
#ifdef VIRTUAL_ROBOT
    static const uint8_t maze[6] = {MAZE_SIZE, GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1, 'V'};
#else
    static const uint8_t maze[5] = {MAZE_SIZE, GOAL_X0, GOAL_Y0, GOAL_X1, GOAL_Y1};
#endif
    return crc32_update(0, maze, sizeof(maze));
}

static uint32_t record_crc(const record_t *r){
    return crc32_update(0, r, offsetof(record_t, crc));
}

static uint8_t params_sane(const params_t *p){
    // Guards against a layout mix-up the CRC could not catch.
    return p->kp >= 0.0f && p->kp <= 100.0f && p->kd >= 0.0f && p->kd <= 20.0f
        && p->search_speed >= SPEED_MIN && p->search_speed <= SPEED_MAX
        && p->fast_speed >= SPEED_MIN && p->fast_speed <= SPEED_MAX
        && p->curve_speed >= SPEED_MIN && p->curve_speed <= SPEED_MAX
        && p->accel >= ACCEL_MIN && p->accel <= ACCEL_MAX
        && p->turn_speed >= TURN_SPEED_MIN && p->turn_speed <= TURN_SPEED_MAX
        && p->turn_accel >= TURN_ACCEL_MIN && p->turn_accel <= TURN_ACCEL_MAX
        && p->turn_ticks >= 300 && p->turn_ticks <= 600
        && p->log_level <= 2 && p->telemetry <= 1;
}

// ---- Slots --------------------------------------------------------------------------------

static uint16_t slot_offset(uint8_t slot){
    return (uint16_t)((slot / SLOTS_PER_PAGE) * FLASH_STORE_PAGE_SIZE + (slot % SLOTS_PER_PAGE) * sizeof(record_t));
}

static const record_t *slot_record(uint8_t slot){
    return (const record_t *)((const uint8_t *)flash_store_data() + slot_offset(slot));
}

static uint8_t erased(const void *p, uint16_t len){
    const uint8_t *b = (const uint8_t *)p;
    for(uint16_t i = 0; i < len; i++){
        if(b[i] != 0xFFu) return 0;
    }
    return 1;
}

static uint8_t slot_free(uint8_t slot){
    return erased(slot_record(slot), sizeof(record_t));
}

// Whole and in this layout (whichever firmware or maze it was for).
static uint8_t slot_valid(uint8_t slot){
    const record_t *r = slot_record(slot);
    return r->magic == STORE_MAGIC && r->version == STORE_VERSION && r->size == sizeof(record_t)
        && r->crc == record_crc(r);
}

// The current record: the valid one with the highest sequence number.
static void find_current(void){
    current = -1;
    for(uint8_t s = 0; s < SLOTS; s++){
        if(!slot_valid(s)) continue;
        const uint32_t seq = slot_record(s)->seq;
        if(current < 0 || (int32_t)(seq - current_seq) > 0){
            current = (int8_t)s;
            current_seq = seq;
        }
    }
}

// An erased slot, going round from the current one.
static int8_t free_slot(void){
    const uint8_t from = current < 0 ? SLOTS - 1u : (uint8_t)current;
    for(uint8_t i = 1; i <= SLOTS; i++){
        const uint8_t s = (uint8_t)((from + i) % SLOTS);
        if(slot_free(s)) return (int8_t)s;
    }
    return -1;
}

static uint8_t page_free(uint8_t page){
    for(uint8_t s = page * SLOTS_PER_PAGE; s < (page + 1u) * SLOTS_PER_PAGE; s++){
        if(!slot_free(s)) return 0;
    }
    return 1;
}

// Programs `record` as the next record into `slot`.
static uint8_t put(uint8_t slot){
    record.seq = current < 0 ? 1u : current_seq + 1u;
    record.crc = record_crc(&record);
    if(!flash_store_program(slot_offset(slot), &record, sizeof(record))) return 0;
    current = (int8_t)slot;
    current_seq = record.seq;
    return 1;
}

// ---- Interface --------------------------------------------------------------------------------

storage_save_t storage_save(uint8_t map_only){
    memset(&record, 0, sizeof(record));
    record.magic = STORE_MAGIC;
    record.version = STORE_VERSION;
    record.size = sizeof(record_t);
    record.seq = current_seq;
    record.maze_signature = maze_defaults_signature();
    record.params_signature = params_defaults_signature();
    maze_export(&record.maze);
    record.params = params;
    record.crc = record_crc(&record);
    // What the current record already holds is not written again.
    if(current >= 0 && memcmp(slot_record((uint8_t)current), &record, sizeof(record)) == 0) return STORAGE_UNCHANGED;
    // Run ends save the map only: switching race presets would spend a slot on each run.
    if(map_only && current >= 0 && memcmp(&slot_record((uint8_t)current)->maze, &record.maze, sizeof(record.maze)) == 0){
        return STORAGE_UNCHANGED;
    }
    int8_t slot = free_slot();
    if(slot < 0) return STORAGE_FULL;
    if(put((uint8_t)slot)) return STORAGE_WRITTEN;
    // A slow halfword the flash recovered from may have landed wrong: once more, in the next slot.
    if(flash_store_blocked() || (slot = free_slot()) < 0) return STORAGE_FAILED;
    return put((uint8_t)slot) ? STORAGE_WRITTEN : STORAGE_FAILED;
}

storage_status_t storage_load(void){
    find_current();
    if(current < 0){
        for(uint8_t s = 0; s < SLOTS; s++){
            const record_t *r = slot_record(s);
            if(r->magic == STORE_MAGIC) return r->version == STORE_VERSION ? STORAGE_CORRUPT : STORAGE_STALE;
        }
        return STORAGE_EMPTY;
    }
    memcpy(&record, slot_record((uint8_t)current), sizeof(record));
    if(!params_sane(&record.params)) return STORAGE_CORRUPT;
    if(record.maze_signature != maze_defaults_signature()) return STORAGE_STALE;
    if(!maze_import(&record.maze)) return STORAGE_CORRUPT;
    // New defaults in the firmware replace the saved parameters; the map stays.
    if(record.params_signature != params_defaults_signature()) return STORAGE_NEW_DEFAULTS;
    params = record.params;
    return STORAGE_LOADED;
}

uint8_t storage_needs_compact(void){
    for(uint8_t s = 0; s < SLOTS; s++){
        if((int8_t)s != current && !slot_free(s)) return 1;
    }
    return 0;
}

// Keeps the current record and frees the rest with at most two page erases, power-cut safe.
uint8_t storage_compact(void){
    if(current < 0){
        for(uint8_t p = 0; p < FLASH_STORE_PAGES; p++){
            if(!page_free(p) && !flash_store_erase(p)) return 0;
        }
        return 1;
    }
    const uint8_t page = (uint8_t)current / SLOTS_PER_PAGE, other = (uint8_t)(1u - page);
    uint8_t crowded = 0;    // the current record shares its page
    for(uint8_t s = page * SLOTS_PER_PAGE; s < (page + 1u) * SLOTS_PER_PAGE; s++){
        if((int8_t)s != current && !slot_free(s)) crowded = 1;
    }
    if(!page_free(other) && !flash_store_erase(other)) return 0;
    if(!crowded) return 1;
    memcpy(&record, slot_record((uint8_t)current), sizeof(record));
    return put(other * SLOTS_PER_PAGE) && flash_store_erase(page);
}

uint8_t storage_free_slots(void){
    uint8_t n = 0;
    for(uint8_t s = 0; s < SLOTS; s++) n = (uint8_t)(n + slot_free(s));
    return n;
}

const char *storage_status_name(storage_status_t status){
    switch(status){
        case STORAGE_LOADED:  return "loaded";
        case STORAGE_NEW_DEFAULTS: return "map loaded, new default parameters";
        case STORAGE_EMPTY:   return "empty";
        case STORAGE_CORRUPT: return "corrupt (ignored)";
        case STORAGE_STALE:   return "from another firmware (ignored)";
    }
    return "?";
}
