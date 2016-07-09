/*
 * file: demo_odometer.ino
 * created: 201600709
 * author(s): mr-augustine
 *
 * This file orchestrates the odometer demo.
 */
#include <stdint.h>

#include "kintobor.h"

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
