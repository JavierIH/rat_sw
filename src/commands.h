#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

// Bluetooth console, one command per line, case-insensitive (README.md); polled from every loop.
void commands_poll(void);

// Non-negative value with two decimals ("30.00"): nano-libc printf has no %f.
const char *format_fixed(char *buf, unsigned size, float v, uint8_t decimals);   // signed, 0-3 decimals
const char *format_fixed2(char *buf, unsigned size, float v);

#endif // COMMANDS_H
