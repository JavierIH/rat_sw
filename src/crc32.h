#ifndef CRC32_H
#define CRC32_H

#include <stddef.h>
#include <stdint.h>

// CRC-32 (IEEE 802.3, reflected, as zlib): 0 to start, the previous result to continue.
uint32_t crc32_update(uint32_t crc, const void *data, size_t len);

#endif // CRC32_H
