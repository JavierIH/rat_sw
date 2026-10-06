#include "crc32.h"

// Bitwise: a table would cost 1 KB of flash for data checksummed only when saving or loading.
uint32_t crc32_update(uint32_t crc, const void *data, size_t len){
    const uint8_t *p = (const uint8_t *)data;
    crc = ~crc;
    while(len--){
        crc ^= *p++;
        for(uint8_t bit = 0; bit < 8; bit++){
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}
