/*
 * file: kintobor.c
 * created: 20160906
 * author(s): mr-augustine
 *
 * The kintobor implementation file defines all of the robot's higher-order
 * functions.
 *
 * This implementation uses the zeroized-whole-degree method in an effort to
 * generate more precise navigation-related values. This mod hopes to address
 * the seemingly insufficient precision given by the float datatype when used
 * to represent the calculated GPS coordinates between measured GPS coordinates.
 */
#include "kintobor.h"
#include <math.h>

#define DEG_TO_RAD(degrees) (degrees * M_PI / 180.0)
#define RAD_TO_DEG(radians) (radians * 180.0 / M_PI)
// Stolen from NOAA: https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015_D_MERC.pdf
// And stolen from NGDC: http://www.ngdc.noaa.gov/geomag-web/
#define MAGNETIC_DECLINATION 8.52  // For Boulder, Colorado
#define MICROS_PER_TICK 4.0
#define SECONDS_PER_TICK 0.000004
#define TICKS_PER_METER 7.6

#define SECONDS_PER_LOOP 0.025

#define TARGET_HEADING 270.0

#define K_PROP 10 // proportional gain
#define K_RATE 0 // derivative gain
#define K_INTEGRAL 0 // integral gain

static float nav_heading_deg = 0.0;
static float rel_bearing_deg = 0.0;
static float waypt_true_bearing = 0.0;

static float calc_nav_heading(void);
static float calc_relative_bearing(float desired_bearing, float current_heading);
static void update_xtrack_error(void);
static void update_xtrack_error_rate(void);
static void update_xtrack_error_sum(void);

static float xtrack_error = 0.0;
static float xtrack_error_prev = 0.0;
static float xtrack_error_rate = 0.0;
static float xtrack_error_sum = 0.0;

static float steer_control = 1500;

static float calc_nav_heading(void) {
  float norm_mag_hdg = statevars.heading_deg + MAGNETIC_DECLINATION;

  if (norm_mag_hdg > 360.0) {
    norm_mag_hdg -= 360.0;
  }

  statevars.nav_heading_deg = norm_mag_hdg;
  // If you just want a compass-based heading, use this
  return norm_mag_hdg;
}

// Calculates the relative bearing in degrees (i.e., the angle between the current
// heading and the waypoint bearing); a negative value means the destination
// is towards the left, and vice versa
// "I'd have to change my heading by this much to point to the waypoint"
static float calc_relative_bearing(float desired_bearing, float current_heading) {
  float diff = desired_bearing - current_heading;

  // We want the range of bearings to be between -180..+180; so a result of
  // -225 (225 degrees to the left of where I'm pointing) will become +135
  // (135 degrees to the right of where I'm pointing)
  if (diff > 180.0) {
    return (diff - 360.0);
  } else if (diff < -180.0) {
    return (diff + 360.0);
  }

  return diff;
}

static void update_xtrack_error(void) {
  // Error = Reference Value - Measured value
  xtrack_error_prev = xtrack_error;

  xtrack_error = calc_relative_bearing(TARGET_HEADING, nav_heading_deg);

  // TODO Change this assignment when you start navigating to waypoints.
  // Normally the desired heading will be the waypt_true_bearing unless we set
  // a new bearing if an obstacle is detected
  statevars.control_heading_desired = TARGET_HEADING;
  // statevars.control_heading_desired = waypt_true_bearing;

  statevars.control_xtrack_error = xtrack_error;
  // TODO Normally the cross track error will be the relative bearing (to the
  // waypoint) unless we decide to set a new bearing if an obstacle is detected
  // statevars.control_xtrack_error = rel_bearing_deg;

  return;
}

static void update_xtrack_error_rate(void) {
  // Rate = (Error - Error_Previous) / Computation Interval
  xtrack_error_rate = (xtrack_error - xtrack_error_prev) / SECONDS_PER_LOOP;

  statevars.control_xtrack_error_rate = xtrack_error_rate;

  return;
}

static void update_xtrack_error_sum(void) {
  // Rate Sum = Rate Sum + Error * Computation Interval
  xtrack_error_sum = xtrack_error_sum + xtrack_error * SECONDS_PER_LOOP;

  statevars.control_xtrack_error_sum = xtrack_error_sum;

  return;
}

void update_all_inputs(void) {
  cmps10_update_all();

  return;
}

void update_nav_control_values(void) {
  nav_heading_deg = calc_nav_heading();

  update_xtrack_error();
  update_xtrack_error_rate();
  update_xtrack_error_sum();

  steer_control = (K_PROP * xtrack_error) +
                  (K_RATE * xtrack_error_rate) +
                  (K_INTEGRAL * xtrack_error_sum);

  // Limit validation for steer_control will be handled by the mobility library.
  // This way we don't command the robot to turn beyond the full left/right
  // steering angles

  // If the xtrack_error is NEGATIVE, then the robot is towards the RIGHT of
  // where it needs to be; If the xtrack error is POSITIVE, then the robot is
  // towards the LEFT of where it needs to be. So, if I'm towards the right of
  // the target heading, I need to turn left. To turn left, you increase the
  // steering PWM value. This is why we have a subtraction in the line below.
  statevars.control_steering_pwm = (uint16_t)(TURN_NEUTRAL - steer_control);
  //statevars.control_steering_pwm = (uint16_t)(statevars.mobility_steering_pwm - steer_control);

  return;
}
