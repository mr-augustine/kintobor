/*
 * file: demo_sdcard_gps_compass_odometer.ino
 * created: 20160710
 * author(s): mr-augustine
 *
 * This file orchestrates the sdcard_gps_compass_odometer demo. When the
 * start/stop button is pressed, GPS data, compass data, and odometer data are
 * continuosly stored to the statevars variable. And the statevars are
 * continuously written to a file on the SD card. The program ends when the
 * start/stop button is pressed again.
 */
#include <stdint.h>

#include "kintobor.h"

/* The MAINLOOP_PERIOD_TICKS value should be some fraction of:
 * 16,000,000 / prescaler
 * Remember: the timer starts counting from zero; so subtract 1
 */
#define MAINLOOP_PERIOD_TICKS   6249    // loop period is 25 ms

statevars_t statevars;
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

  clear_statevars();
  statevars.prefix = 0xDADAFEED;
  statevars.suffix = 0xCAFEBABE;

  iterations = 0;

  configure_mainloop_timer();
  enable_mainloop_timer();

  // TODO Consider functionalizing this pre-mission hold and place it with the
  // other higher-order functions
  // Don't start the mission until the start/stop button is pressed
  do {
    led_turn_on();
    uwrite_print_buff("Waiting for button to be pressed\r\n");
    button_update();
  } while (!button_is_pressed());

  if (button_is_pressed()) {
    led_turn_off();
    uwrite_print_buff("Mission started!\r\n");
    // TODO Consider calling update_all_input() before starting the main loop
    // so that the first block written isn't empty
  }
}

void loop() {
  TCNT1 = 0;

  // Take note if the system timer (TIMER1) had an overflow
  if (system_timer_overflow) {
    statevars.status |= STATUS_SYS_TIMER_OVERFLOW;
  }

  // Record the data from the previous iteration
  write_data();

  // Reset statevars and timer overflow flag
  statevars.status = 0;

  statevars.main_loop_counter = iterations;
  system_timer_overflow = 0;

  update_all_inputs();


  iterations++;

  // If the button switched to the OFF position, then stop the mission
  if (!button_is_pressed()) {
    uwrite_print_buff("Finished collecting data!\r\n");
    sdcard_finish();

    exit(0);
  }

  /* Ensure that the main loop period is as long as we want it to be.
   * This means (1) triggering the main loop to restart when we notice it is
   * running too long, and (2) performing busy waiting if the instructions
   * above finish before the desired loop duration.
   */
   if (TCNT1 > MAINLOOP_PERIOD_TICKS) {
     statevars.status |= STATUS_MAIN_LOOP_LATE;

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

void clear_statevars(void) {
  memset(&statevars, 0, sizeof(statevars));
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

// TODO Shift this function into the higher-order functions group, but
// separate the sdcard_init() since it won't be recognized by the AVR code
uint8_t init_all_subsystems(void) {
  uwrite_init();

  if (!button_init()) {
    uwrite_print_buff("LED button couldn't be initialized\r\n");
    return 0;
  } else {
    uwrite_print_buff("LED button is ready!\r\n");
    led_turn_off();
  }

  if (!cmps10_init()) {
    uwrite_print_buff("Compass couldn't be initialized\r\n");
    return 0;
  } else {
    uwrite_print_buff("Compass is ready!\r\n");
  }

  if (!gps_init()) {
    uwrite_print_buff("GPS sensor couldn't be initialized\r\n");
    return 0;
  } else {
    uwrite_print_buff("GPS sensor is ready!\r\n");
  }

  if (!odometer_init()) {
    uwrite_print_buff("Odometer couldn't be initialized\r\n");
    return 0;
  } else {
    uwrite_print_buff("Odometer is ready!\r\n");
  }
  
  if (!sdcard_init(&statevars, sizeof(statevars))) {
    uwrite_print_buff("SD card couldn't be initialized\r\n");
    return 0;
  } else {
    uwrite_print_buff("SD card is ready!\r\n");
  }

  return 1;
}
