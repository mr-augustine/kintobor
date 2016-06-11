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

statevars_t statevars;

void setup() {
  Serial.begin(115200);

  if (sdcard_init(&statevars, sizeof(statevars))) {
    Serial.println("All systems ready!");
  } else {
    Serial.println("There was a subsystem failure");
  }

  statevars.prefix = 0xDADAFEED;
  statevars.suffix = 0xCAFEBABE;
  statevars.status = 8675309;

  write_data();
  sdcard_finish();
}

void loop() {

}
