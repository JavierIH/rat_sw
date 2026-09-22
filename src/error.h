#ifndef ERROR_H
#define ERROR_H

// Fatal HAL initialisation error: stops the motors, reports over the console
// and blinks all LEDs fast forever.
void Error_Handler(void);

#endif // ERROR_H
