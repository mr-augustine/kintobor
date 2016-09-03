/*
 * file: kintobor.c
 * created: 20160902
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
#define EARTH_RADIUS_M 6371393.0
// Stolen from NOAA: https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015_D_MERC.pdf
#define MAGNETIC_DECLINATION 4.0  // For central Texas
#define METERS_PER_SECOND_PER_KNOT 0.514444
#define MICROS_PER_TICK 4.0
#define SECONDS_PER_TICK 0.000004
#define TICKS_PER_METER 7.6

static float current_lat;
static float current_long;
static float waypoint_lat;
static float waypoint_long;
static float nav_heading_deg;
static float rel_bearing_deg;
static float distance_to_waypoint_m;
static float current_speed; // in meters per second

static float last_gps_heading_deg;
static float last_gps_speed; // in meters per second
static uint32_t prev_tick_count;

static float calc_dist_to_waypoint(float start_lat, float start_long, float end_lat, float end_long);
static float calc_mid_angle(float heading_1, float heading_2);
static float calc_nav_heading(void);
static void calc_position(float* new_lat, float* new_long, float ref_lat, float ref_long, float distance, float heading);
static float calc_relative_bearing(float start_lat, float start_long, float dest_lat, float dest_long, float heading);
static float calc_speed(float distance_m);
static float calc_speed_mps(uint32_t ticks);
static void get_next_waypoint(void);

static uint8_t got_first_coord = 0;

// Returns the distance (in meters) to the current waypoint
static float calc_dist_to_waypoint(float lat_1, float long_1, float lat_2, float long_2) {
  float lat_1_rad = DEG_TO_RAD(lat_1);
  float long_1_rad = DEG_TO_RAD(long_1);
  float lat_2_rad = DEG_TO_RAD(lat_2);
  float long_2_rad = DEG_TO_RAD(long_2);

  float diff_lat = lat_2_rad - lat_1_rad;
  float diff_long = long_2_rad - long_1_rad;

  float a = ( pow(sin(diff_lat / 2), 2) ) +
    cos(lat_1_rad) *
    cos(lat_2_rad) *
    ( pow(sin(diff_long / 2), 2) );

  float c = 2 * asin(sqrt(a));

  float distance_m = EARTH_RADIUS_M * c;

  return distance_m;
}

// Returns the angle that is halfway between the specified headings
static float calc_mid_angle(float heading_1, float heading_2) {
  // Ensure that heading_2 stores the larger heading
  if (heading_1 > heading_2) {
    float temp = heading_1;
    heading_1 = heading_2;
    heading_2 = temp;
  }

  if (heading_2 - heading_1 > 180.0) {
    heading_2 -= 360.0;
  }

  float mid_angle = (heading_2 + heading_1) / 2.0;

  if (mid_angle < 0.0) {
    mid_angle += 360.0;
  }

  return mid_angle;
}

static float calc_nav_heading(void) {
  float norm_mag_hdg = statevars.heading_deg + MAGNETIC_DECLINATION;

  // Here we're calculating the navigation heading as the mid-angle between
  // the compass heading and the GPS heading because experimental data seemed
  // to produce good results when we did this.
  float nav_heading = calc_mid_angle(norm_mag_hdg, last_gps_heading_deg);

  // return norm_mag_hdg;
  return nav_heading;
}

// Calculates a new position based on the current heading and distance
// traveled from the previous position
static void calc_position(float* new_lat, float* new_long, float ref_lat, float ref_long, float distance, float heading) {
  float lat_rad = DEG_TO_RAD(ref_lat);
  float long_rad = DEG_TO_RAD(ref_long);
  float heading_rad = DEG_TO_RAD(heading);

  float est_lat = asin( sin(lat_rad) *
    cos(distance / EARTH_RADIUS_M) +
    cos(lat_rad) *
    sin(distance / EARTH_RADIUS_M) *
    cos(heading_rad) );

  float est_long = long_rad +
    atan2( sin(heading_rad) *
    sin(distance / EARTH_RADIUS_M) *
    cos(lat_rad),
    cos(distance / EARTH_RADIUS_M) -
    sin(lat_rad) *
    sin(est_lat) );

  *new_lat = RAD_TO_DEG(est_lat);
  *new_long = RAD_TO_DEG(est_long);

  return;
}

// Calculates the relative bearing (i.e., the angle between the current
// heading and the waypoint bearing); a negative value means the destination
// is towards the left, and vice versa
static float calc_relative_bearing(float start_lat, float start_long, float dest_lat, float dest_long, float heading) {
  float start_lat_rad = DEG_TO_RAD(start_lat);
  float start_long_rad = DEG_TO_RAD(start_long);
  float dest_lat_rad = DEG_TO_RAD(dest_lat);
  float dest_long_rad = DEG_TO_RAD(dest_long);

  float y = sin(dest_long_rad - start_long_rad) *
    cos(dest_lat_rad);
  float x = cos(start_lat_rad) *
    sin(dest_lat_rad) -
    sin(start_lat_rad) *
    cos(dest_lat_rad) *
    cos(dest_long_rad - start_long_rad);

  float bearing_rad = atan2(y, x);
  float bearing_deg = RAD_TO_DEG(bearing_rad);

  return (bearing_deg - heading);
}

// Calculate the robot's current speed based on how many odometer ticks were
// measured; result is in meters per second
static float calc_speed_mps(uint32_t ticks) {
  if (ticks == 0) {
    return 0.0;
  }

  float distance_m = ticks / TICKS_PER_METER;
  float elapsed_time_s = statevars.odometer_timestamp * SECONDS_PER_TICK;

  float speed_meters_per_sec = 0.0;

  if (elapsed_time_s > 0.0) {
    speed_meters_per_sec = distance_m / elapsed_time_s;
  }

  return speed_meters_per_sec;
}

// Calculates the robot's current speed; result in meters per second
static float calc_speed(float distance_m) {
  float elapsed_time_s = statevars.odometer_timestamp * MICROS_PER_TICK / 1000000.0;

  float speed_meters_per_sec = 0.0;

  if (elapsed_time_s > 0.0) {
    speed_meters_per_sec = distance_m / elapsed_time_s;
  }

  return speed_meters_per_sec;
}

// Gets the next waypoint
// For this demo, we're using the first GPS coordinate we received
static void get_next_waypoint(void) {
  if (got_first_coord == 1) {
    return;
  }

  if (statevars.status & STATUS_GPS_GPGGA_RCVD) {
    waypoint_lat = statevars.gps_latitude;
    waypoint_long = statevars.gps_longitude;

    got_first_coord = 1;
  }

  return;
}

static void update_all_nav(void) {
  get_next_waypoint();

  // Check if a new GPS coordinate was received and update the position
  if (statevars.status & STATUS_GPS_GPGGA_RCVD) {
    current_lat = statevars.gps_lat_ddeg;
    current_long = statevars.gps_long_ddeg;
  }

  // Check if a new GPS heading and speed were received and update
  if (statevars.status & STATUS_GPS_GPRMC_RCVD) {
    last_gps_heading_deg = statevars.gps_ground_course_deg;
    last_gps_speed = statevars.gps_ground_speed_kt * METERS_PER_SECOND_PER_KNOT;
  }

  // Calculate the number of this that occurred during the current iteration
  // Since the tick count is cumulative, the new tick count will always be
  // greater-than or equal to the previous tick count
  uint32_t new_tick_count = statevars.odometer_ticks;
  uint32_t tick_diff = new_tick_count - prev_tick_count;

  current_speed = calc_speed_mps(tick_diff);

  // TODO I know; we're doing another tick_diff / TICKS_PER_METER calculation.
  // But we'll optimize this later
  float distance_since_prev_iter_m = tick_diff / TICKS_PER_METER;

  // We're using calc_speed_mps() instead to allow integer-based distance diff eval
  // current_speed = calc_speed(distance_since_prev_iter_m);

  // Advance the tick count now that we're done with the previous value
  prev_tick_count = new_tick_count;

  if (current_speed == 0.0) {
    current_speed = last_gps_speed;
  }

  nav_heading_deg = calc_nav_heading();

  float old_lat = current_lat;
  float old_long = current_long;
  calc_position(&current_lat, &current_long, old_lat, old_long, distance_since_prev_iter_m, nav_heading_deg);

  distance_to_waypoint_m = calc_dist_to_waypoint(current_lat, current_long, waypoint_lat, waypoint_long);
  rel_bearing_deg = calc_relative_bearing(current_lat, current_long, waypoint_lat, waypoint_long, nav_heading_deg);

  statevars.nav_heading_deg = nav_heading_deg;
  statevars.nav_latitude = current_lat;
  statevars.nav_longitude = current_long;
  statevars.nav_waypt_latitude = waypoint_lat;
  statevars.nav_waypt_longitude = waypoint_long;
  statevars.nav_rel_bearing_deg = rel_bearing_deg;
  statevars.nav_distance_to_waypt_m = distance_to_waypoint_m;
  statevars.nav_speed = current_speed;

  return;
}

void update_all_inputs(void) {
  button_update();
  cmps10_update_all();
  gps_update();
  odometer_update();

  update_all_nav();

  return;
}
