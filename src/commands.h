#ifndef COMMANDS_H
#define COMMANDS_H

// Bluetooth command console, one command per line, case-insensitive. HELP
// lists them. Polled from the idle loop and from every motion loop.
void commands_poll(void);

#endif // COMMANDS_H
