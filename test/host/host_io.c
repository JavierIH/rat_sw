// Host stand-ins for the console functions search.c uses.
#include <stdarg.h>
#include <stdio.h>
#include "uart.h"

int host_verbose;

void print(const char *format, ...){
    char text[256];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);    // always formatted: sanitizers see bad args
    va_end(args);
    if(host_verbose) fputs(text, stdout);
}

void uart_wait_space(uint32_t timeout_ms){
    (void)timeout_ms;
}
