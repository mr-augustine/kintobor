/*
 * file: kintobor.c
 * created: 20160808
 * author(s): mr-augustine
 *
 * The kintobor implementation file defines all of the robot's higher-order
 * functions.
 */
#include "kintobor.h"

void update_all_inputs(void) {
  button_update();
  cmps11_update_all();
  gps_update();
  odometer_update();

  return;
}
