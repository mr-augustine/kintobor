/*
 * file: kintobor.h
 * created: 20160612
 * author(s): mr-augustine
 *
 * The kintobor header file lists function prototypes for all of the robot's
 * higher-order functions.
 */
#ifndef _KINTOBOR_H_
#define _KINTOBOR_H

#include "cmps10.h"
#include "gps.h"
#include "ledbutton.h"
#include "odometer.h"
#include "statevars.h"
#include "uwrite.h"

#define ROBOT_NAME ("kintobor")

// typedef struct {
//   uint32_t timestamp;
//   uint32_t ticks;
// } Timestamped_Tick;
//
// Timestamped_Tick prev_tick;  // timestamped tick from the previous iteration
// Timestamped_Tick curr_tick;  // timestamped tick for the current iteration
// TODO Consider getting the timestamps using the micros() function

#ifdef __cplusplus
extern "C" {
  void update_all_inputs(void);

  // Odometer-related higher-order functions
  // Calculates the distance traveled since the last iteration; result in meters
  // float calc_iter_dist(void);

  // Calculates the cumulative distance traveled in the forward direction since
  // mission start
  // float calc_total_fwd_dist(void);

  // Calculates the cumulative distance traveled in the reverse direction since
  // mission start
  // float calc_total_rev_dist(void);

  // Wrapper for the above two functions; access the distance you want directly
  // from the variable (total_fwd_distance, total_rev_distance)
  // void calc_total_distances(void);

  // Calculates the robot's current speed; result in meters per second
  // float calc_speed(void);
}
#endif // #ifdef __cplusplus

#endif // #ifndef _KINTOBOR_H_
