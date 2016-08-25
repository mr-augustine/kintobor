/*
 * file: kintobor.c
 * created: 20160824
 * author(s): mr-augustine
 *
 * The kintobor implementation file defines all of the robot's higher-order
 * functions.
 */
#include "kintobor.h"
#include <math.h>

#define DEG_TO_RAD(degrees) (degrees * M_PI / 180.0)
#define RAD_TO_DEG(radians) (radians * 180.0 / M_PI)
#define EARTH_RADIUS_M 6371393.0
// Stolen from NOAA: https://www.ngdc.noaa.gov/geomag/WMM/data/WMM2015/WMM2015_D_MERC.pdf
#define MAGNETIC_DECLINATION 4.0  // For central Texas
#define MICROS_PER_TICK 4.0
#define TICKS_PER_METER 7.6

static float current_lat;
static float current_long;
static float waypoint_lat;
static float waypoint_long;
static float nav_heading_deg;
static float rel_bearing_deg;
static float distance_to_waypoint_m;
static float last_gps_heading_deg;
static uint32_t prev_tick_count;
static uint8_t num_ticks;

static float calc_dist_to_waypoint(float start_lat, float start_long, float end_lat, float end_long);
static float calc_mid_angle(float heading_1, float heading_2);
static float calc_nav_heading(void);
static void calc_position(float* new_lat, float* new_long, float ref_lat, float ref_long, float distance, float heading);
static float calc_relative_bearing(float start_lat, float start_long, float dest_lat, float dest_long, float heading);
static float calc_speed(void);
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

  float a = (sin(diff_lat / 2) ^ 2) +
    cos(lat_1_rad) *
    cos(lat_2_rad) *
    (sin(diff_long / 2) ^ 2);

  float c = 2 * asin(sqrt(a));

  float distance_m = EARTH_RADIUS_M * c;

  return distance_m;
}

// Returns the angle that is halfway between the specified headings
static float calc_mid_angle(float heading_1, float heading_2) {
  float hdg_1_rad = DEG_TO_RAD(heading_1);
  float hdg_2_rad = DEG_TO_RAD(heading_2);

  // avr-gcc compiles with gnu++1 and gnu11 standard; the trig functions accept
  // float params
  float resultant[2] = {0.0, 0.0};
  float resultant[0] = cos(hdg_1_rad) + cos(hdg_2_rad);
  float resultant[1] = sin(hdg_1_rad) + sin(hdg_2_rad);

  float mid_angle_rad = atan2(resultant[1], resultant[0]);
  float mid_angle_deg = RAD_TO_DEG(mid_angle_rad);

  if mid_angle_deg < 0.0 {
    mid_angle_deg += 360.0;
  }

  return mid_angle_deg;
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

// Calculates the robot's current speed; result in meters per second
static float calc_speed(void) {
  float elapsed_time_s = statevars.odometer_timestamp * MICROS_PER_TICK / 1000000.0;

  float speed_meters_per_sec = 0.0;

  if (elapsed_time_s > 0.0) {
    speed_meters_per_sec = num_ticks / TICKS_PER_METER / elapsed_time_s;
  }

  return speed_meters_per_sec;
}

// Gets the next waypoint
// For this demo, we're using the first GPS coordinate we received
static void get_next_waypoint(void) {
  if (got_first_coord == 1) {
    return;
  }

  if (statevars.status & (1 << STATUS_GPS_GPGGA_RCVD) == 1) {
    waypoint_lat = statevars.latitude;
    waypoint_long = statevars.longitude;

    got_first_coord = 1;
  }

  return;
}

// TODO NEED TO GET ALL RELEVANT GPS VALUES UPDATED!
// TODO calculate the number of ticks since the last iteration
static void update_all_nav(void) {
  // Get the latest lat/long if available
  if (got_first_coord == 0) {
    float svars_lat = statevars.latitude;
    float svars_long = statevars.longitude;

    // For this demo, set the very first GPS coordinate as the waypoint
    if (svars_lat != 0.0 && svars_long != 0.0) {
      current_pos.latitude = svars_lat;
      current_pos.longitude = svars_long;

      waypoint_pos.latitude = svars_lat;
      waypoint_pos.longitude = svars_long;

      got_first_coord == 1;
    }
  }

  nav_heading_deg = calc_nav_heading();

  if (got_first_coord == 1) {
    // if a new GPS update occurred use that
    // otherwise use the most recent calculated position
  } else {}

  statevars.nav_heading_deg = nav_heading_deg;
  statevars.calc_latitude = current_pos.latitude;
  statevars.calc_longitude = current_pos.longitude;
  statevars.waypoint_latitude = waypoint_pos.latitude;
  statevars.waypoint_longitude = waypoint_pos.longitude;
  statevars.relative_bearing_deg = relative_bearing_deg;
  statevars.distance_to_waypoint_m = distance_to_waypoint_m;

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
