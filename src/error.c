#include "error.h"

void Error_Handler(void){
    char tbuf[100];
    sprintf(tbuf,"ERROR HANDLER\n\r");
    send_uart(tbuf);
}
