#include "storage.h"
#include <stddef.h>
#include <string.h>
#include "crc32.h"
#include "flash_store.h"
#include "maze.h"
#include "params.h"

#define STORE_MAGIC     0x4D544152u     // "RATM"
#define STORE_VERSION   4u      // 4: speeds in mm/s and deg/s

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t signature;     // params_defaults_signature() of the firmware that saved it
    maze_snapshot_t maze;
    params_t params;
    uint32_t crc;           // CRC-32 of every byte before this field
} record_t;

_Static_assert(sizeof(record_t) % 4 == 0, "flash is programmed in 32-bit words");
_Static_assert(sizeof(record_t) <= FLASH_STORE_SIZE, "record does not fit in the flash page");
_Static_assert(offsetof(record_t, crc) == sizeof(record_t) - 4, "crc must be the last field");

static record_t record;     // static: too big for the 1 KB stack budget

static uint32_t record_crc(const record_t *r){
    return crc32_update(0, r, offsetof(record_t, crc));
}

static uint8_t params_sane(const params_t *p){
    // Written by us with range-checked values; this only guards against a
    // layout mix-up that the CRC could not catch.
    return p->kp >= 0.0f && p->kp <= 10.0f && p->ki >= 0.0f && p->ki <= 100.0f
        && p->search_speed >= SPEED_MIN && p->search_speed <= SPEED_MAX
        && p->fast_speed >= SPEED_MIN && p->fast_speed <= SPEED_MAX
        && p->accel >= ACCEL_MIN && p->accel <= ACCEL_MAX
        && p->turn_speed >= TURN_SPEED_MIN && p->turn_speed <= TURN_SPEED_MAX
        && p->turn_accel >= TURN_ACCEL_MIN && p->turn_accel <= TURN_ACCEL_MAX
        && p->turn_ticks >= 300 && p->turn_ticks <= 600
        && p->log_level <= 2 && p->telemetry <= 1;
}

uint8_t storage_save(void){
    memset(&record, 0, sizeof(record));
    record.magic = STORE_MAGIC;
    record.version = STORE_VERSION;
    record.size = sizeof(record_t);
    record.signature = params_defaults_signature();
    maze_export(&record.maze);
    record.params = params;
    record.crc = record_crc(&record);
    return flash_store_write(&record, sizeof(record));
}

storage_status_t storage_load(void){
    memcpy(&record, flash_store_data(), sizeof(record));
    if(record.magic != STORE_MAGIC) return STORAGE_EMPTY;
    if(record.version != STORE_VERSION || record.size != sizeof(record_t)) return STORAGE_STALE;
    if(record.crc != record_crc(&record) || !params_sane(&record.params)) return STORAGE_CORRUPT;
    if(record.signature != params_defaults_signature()) return STORAGE_STALE;
    if(!maze_import(&record.maze)) return STORAGE_CORRUPT;
    params = record.params;
    return STORAGE_LOADED;
}

const char *storage_status_name(storage_status_t status){
    switch(status){
        case STORAGE_LOADED:  return "cargado";
        case STORAGE_EMPTY:   return "vacio";
        case STORAGE_CORRUPT: return "corrupto (ignorado)";
        case STORAGE_STALE:   return "de otro firmware (ignorado)";
    }
    return "?";
}
