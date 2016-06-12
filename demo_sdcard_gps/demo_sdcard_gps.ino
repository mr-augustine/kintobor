/*
 * file: demo_sdcard_gps.ino
 * created: 20160611
 * author(s): mr-augustine
 *
 * This file orchestrates the sdcard_gps demo.
 *
 * For now, the program will write a statevars entry to a hard-coded file.
 */
#include <stdint.h>

#include "kintobor.h"
#include "statevars.h"

/* The MAINLOOP_PERIOD_TICKS value should be some fraction of:
 * 16,000,000 / prescaler
 * Remember: the timer starts counting from zero; so subtract 1
 */
#define MAINLOOP_PERIOD_TICKS   6249    // loop period is 25 ms

statevars_t statevars;

volatile uint8_t mainloop_timer_overflow = 0; // indicates mainloop overflow
uint32_t iterations;
/* Interrupt Service Routine that triggers when the main loop is running longer
 * than it should.
 */
ISR(TIMER1_OVF_vect) {
  mainloop_timer_overflow = 1;
}

void setup() {
  Serial.begin(9600);

  if (sdcard_init(&statevars, sizeof(statevars))) {
    Serial.println("All systems ready!");
  } else {
    Serial.println("There was a subsystem failure");
  }

  clear_statevars();
  statevars.prefix = 0xDADAFEED;
  statevars.suffix = 0xCAFEBABE;

  iterations = 0;

  configure_mainloop_timer();
  enable_mainloop_timer();

  //write_data();
  //sdcard_finish();
}

void loop() {
  TCNT1 = 0;
  statevars.status = 0;

  statevars.main_loop_counter = iterations;
  mainloop_timer_overflow = 0;

  write_data();

  iterations++;

  if (iterations > 256) {
    Serial.println("Finished collecting data!");
    sdcard_finish();

    exit(0);
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
