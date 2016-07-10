/*
 * file: demo_odometer.ino
 * created: 201600709
 * author(s): mr-augustine
 *
 * This file orchestrates the odometer demo. When the odometer detects a wheel
 * rotation, a message is printed.
 */
#include <stdint.h>

#include "kintobor.h"

/* The MAINLOOP_PERIOD_TICKS value should be some fraction of:
 * 16,000,000 / prescaler
 * Remember: the timer starts counting from zero; so subtract 1
 */
#define MAINLOOP_PERIOD_TICKS   6249    // loop period is 25 ms

uint32_t iterations;

volatile uint8_t system_timer_overflow = 0; // indicates mainloop overflow
/* Interrupt Service Routine that triggers when the main loop is running longer
 * than it should.
 */
ISR(TIMER1_OVF_vect) {
  system_timer_overflow = 1;
}

void setup() {
  if (init_all_subsystems()) {
    uwrite_print_buff("All systems ready!\r\n");
  } else {
    uwrite_print_buff("There was a subsystem failure\r\n");
    exit(0);
  }

  iterations = 0;

  configure_mainloop_timer();
  enable_mainloop_timer();
}


void loop() {
  TCNT1 = 0;

  system_timer_overflow = 0;

  iterations++;

  // Nothing to see here; the print message is handled by the interrupt
  // service routine in odometer.c
  //uint32_t ticks = odometer_get_fwd_count();
  //uwrite_print_buff("ticks: ");
  //uwrite_println_long(&ticks);

  /* Ensure that the main loop period is as long as we want it to be.
  * This means (1) triggering the main loop to restart when we notice it is
  * running too long, and (2) performing busy waiting if the instructions
  * above finish before the desired loop duration.
  */
  if (TCNT1 > MAINLOOP_PERIOD_TICKS) {
   //statevars.status |= STATUS_MAIN_LOOP_LATE;

   // Jump to the start of loop() by calling return. Normally we would use
   // continue to go to the beginning of a loop, but in this case, loop() is
   // a function. And when you return from loop() it gets called again by
   // main.cpp.
   return;
  }

  while (1) {
    // System timer reached 250,000
    if (system_timer_overflow) {
      break;
    }

    // Main loop timer reached 6250
    if (TCNT1 > MAINLOOP_PERIOD_TICKS) {
      break;
    }
  }

}

void configure_mainloop_timer(void) {
  cli();

  /* We need to use a timer to control how long our main loop iterations are.
   * Here we are configuring Timer1 because it is a "16-bit" timer, which would
   * allow use to count to values greater than the usual (2^8)-1. For a
   * prescale value of 64, the timer will count up to 16,000,000/64 = 250,000.
   * This gives us a timer period of 250,000 ticks/sec = 4 microseconds/tick.
   * Suppose we want the main loop to run 40 times per second (40 Hz), we would
   * need to restart the loop when Timer1 reaches the value: 250,000/40 = 6250.
   * See Atmel datasheet for Mega, Section 17.11
   */
  TCCR1A = 0b00000000; // Normal operation; no waveform generation by default
  TCCR1B = 0b00000011; // No input capture, waveform gen; prescaler = 64
  TCCR1C = 0b00000000; // No output compare
  TIMSK1 = 0b00000000; // Ensure overflow interrupt is disabled for now

  sei();

  return;
}

void enable_mainloop_timer(void) {
  TIMSK1 = 0b00000001;

  return;
}

uint8_t init_all_subsystems(void) {
  uwrite_init();

  if (!odometer_init()) {
    uwrite_print_buff("Odometer couldn't be initialized\r\n");
    return 0;
  } else {
    uwrite_print_buff("Odometer is ready!\r\n");
  }

  return 1;
}
