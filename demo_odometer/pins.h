/*
 * file: pins.h
 * created: 20160708
 * author(s): mr-augustine
 *
 * Defines the pins used on the Arduino Mega.
 *
 * Every physical wire connected to the Arduino is plugged into a pin.
 * And for each occupied pin, we define:
 *   (1) the PORT it is on
 *   (2) the Data Direction Register associated with that PORT
 *   (3) the PORT's Pin Vector
 *   (4) the Pin Vector address for the bit associated with the physical pin
 */
#ifndef _PINS_H_
#define _PINS_H_

#include <avr/io.h>                 // For the pin names (e.g., PB2)

////////////////////////////////////////////////////////////////////////////////
// ODOMETER
#define ODOMETER_PORT         PORTD
#define ODOMETER_DDR          DDRD
#define ODOMETER_PINVEC       PIND
#define ODOMETER_PIN          PD2   // Mega Digital Pin 19
// TODO Switch the GPS pins to something else; say 16 and 17

////////////////////////////////////////////////////////////////////////////////
// USART WRITE
// Keep these pins unoccupied       // Mega Digital Pin 0
                                    // Mega Digital Pin 1

#endif // #ifndef _PINS_H_
