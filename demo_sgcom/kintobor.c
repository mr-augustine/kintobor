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
  cmps10_update_all();
  gps_update();

  // TODO This function may need to be wrapped in another function which also
  // handles timestamp updates (for distance and speed calculations)
  odometer_update();

  return;
}
