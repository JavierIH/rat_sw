#ifndef CRC32_H
#define CRC32_H

#include <stddef.h>
#include <stdint.h>

// Standard CRC-32 (IEEE 802.3, reflected, as used by zlib). Pass 0 as `crc`
// to start; feed the previous result to continue over more data.
uint32_t crc32_update(uint32_t crc, const void *data, size_t len);

#endif // CRC32_H
