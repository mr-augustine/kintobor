/*
 * file: odometer.c
 * created: 20160709
 * author(s): mr-augustine
 *
 * Defines the functions used to operate the odometer. This library counts
 * the number rotations performed by the robot's drive gear; and stores the
 * value in a statevars variable.
 *
 * The rotations are detected by a hall effect sensor which observes a magnet
 * affixed to the drive gear.
 */
#include <avr/interrupt.h>
#include <avr/io.h>

#include "odometer.h"
#include "uwrite.h"

volatile static uint32_t fwd_count;
volatile static uint32_t rev_count;
static Wheel_Direction wheel_turn_direction;

ISR(ODOMETER_ISR_VECT) {
  // TODO Decide if we should only increment when the mission has started
  // Increment the approriate count variable (fwd_count or rev_count)
  if (wheel_turn_direction == Direction_Forward) {
    fwd_count++;
    uwrite_println_long(&fwd_count);
  } else {
    rev_count++;
  }
}

uint8_t odometer_init(void) {
  // Turn on the pull-up resistor for the odometer pin
  ODOMETER_PORT |= (1 << ODOMETER_PIN);

  // Set the odometer pin as an input
  ODOMETER_DDR &= ~(1 << ODOMETER_PIN);

  // Set the External Interrupt to Falling Edge
  // The odometer pin is normally high when the magnet is not present, and then
  // becomes low when the magnet passes in front of it.
  // See Table 15-1 in the Atmel specs
  EICRA &= ~(1 << ISC20);
  EICRA |= (1 << ISC21);

  // Enable interrupts on odometer pin
  EIMSK |= (1 << ODOMETER_INTERRUPT_MASK_PIN);

  odometer_reset();
  wheel_turn_direction = Direction_Forward;

  return 1;
}

void odometer_reset(void) {
    odometer_reset_fwd_count();
    odometer_reset_rev_count();

    return;
}

void odometer_reset_fwd_count(void) {
  fwd_count = 0;

  return;
}

void odometer_reset_rev_count(void) {
  rev_count = 0;

  return;
}

void odometer_set_direction(Wheel_Direction wd) {
  wheel_turn_direction = wd;

  return;
}

uint32_t odometer_get_fwd_count(void) {
  return fwd_count;
}

uint32_t odometer_get_rev_count(void) {
  return rev_count;
}
