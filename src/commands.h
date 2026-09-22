#ifndef COMMANDS_H
#define COMMANDS_H

// Bluetooth command console, one command per line, case-insensitive. HELP
// lists them. Polled from the idle loop and from every motion loop.
void commands_poll(void);

// Non-negative value with two decimals ("30.00"): nano-libc printf has no %f.
const char *format_fixed2(char *buf, unsigned size, float v);

#endif // COMMANDS_H
