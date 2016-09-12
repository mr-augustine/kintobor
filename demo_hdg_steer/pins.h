/*
 * file: pins.h
 * created: 20160912
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
// COMPASS (CMPS10)
#define COMPASS_PORT        PORTD
#define COMPASS_DDR         DDRD
#define COMPASS_PINVEC      PIND
#define COMPASS_SDA_PIN     PD1     // Mega Digital Pin 20
#define COMPASS_SCL_PIN     PD0     // Mega Digital Pin 21

////////////////////////////////////////////////////////////////////////////////
// MOBILITY
#define STEERING_PORT       PORTE
#define STEERING_DDR        DDRE
#define STEERING_PINVEC     PINE
#define STEERING_PIN        PE4     // Mega Digital Pin 2

#define THROTTLE_PORT       PORTE
#define THROTTLE_DDR        DDRE
#define THROTTLE_PINVEC     PINE
#define THROTTLE_PIN        PE5     // Mega Digital Pin 3

////////////////////////////////////////////////////////////////////////////////
// USART WRITE
// Keep these pins unoccupied       // Mega Digital Pin 0
                                    // Mega Digital Pin 1

#endif // #ifndef _PINS_H_
