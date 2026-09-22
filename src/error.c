#include "error.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "motor.h"
#include "uart.h"

void Error_Handler(void){
    motor_emergency_stop();
    uart_send("ERROR HANDLER\r\n");
    for(;;) leds_blink(1, 50);
}
