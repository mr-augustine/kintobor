/*
 * file: odometer.h
 * created: 20160709
 * author(s): mr-augustine
 *
 * Lists the functions used to initialize and update the odometer.
 *
 * The extern "C" construct allows the main Arduino program to use the
 * functions declared below.
 */
#ifndef _ODOMETER_H_
#define _ODOMETER_H_

#include <avr/io.h>

#define ODOMETER_ISR_VECT             INT2_vect
#define ODOMETER_INTERRUPT_MASK_PIN   INT2

typdef enum {
  Direction_Forward,
  Direction_Reverse
} Wheel_Direction;

#ifdef __cplusplus
extern "C" {
  uint8_t odometer_init(void);
  void odometer_reset(void);
  void odometer_reset_fwd_count(void);
  void odometer_reset_rev_count(void);
  void odometer_set_direction(Wheel_Direction wd);
  uint32_t odometer_get_fwd_count(void);
  uint32_t odometer_get_rev_count(void);
}
#endif // #ifdef __cplusplus

#endif // #ifndef _GPS_H_
