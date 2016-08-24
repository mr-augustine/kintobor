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

static GPS_Coordinate current_pos;
static GPS_Coordinate waypoint_pos;
static float nav_heading_deg;
static float rel_bearing_deg;
static float distance_to_waypoint_m

static float calc_dist_to_waypoint(GPS_Coordinate* curr_position, GPS_Coordinate* way_position);
static float calc_mid_angle(float heading_1, float heading_2);
static float calc_nav_heading(void);
static GPS_Coordinate calc_position(GPS_Coordinate* prev_position, float distance, float heading);
static float calc_speed(void);
static GPS_Coordinate get_next_waypoint(void);
static float degrees_to_radians(float degrees);
static float radians_to_degrees(float radians);


uint8_t got_first_coord = 0;

// Returns the distance (in meters) to the current waypoint
static float calc_dist_to_waypoint(GPS_Coordinate* curr_position, GPS_Coordinate* way_position) {
  float lat1_rad = DEG_TO_RAD(curr_position.latitude);
  float long1_rad = DEG_TO_RAD(curr_position.longitude);
  float lat2_rad = DEG_TO_RAD(way_position.latitude);
  float long2_rad = DEG_TO_RAD(way_position.longitude);

  float dlat = lat2_rad - lat1_rad;
  float dlong = long2_rad - long1_rad;

  float a = (sin(dlat / 2) ^ 2) + cos(lat1_rad) * cos(lat2_rad) * (sin(dlong / 2) ^ 2);
  float c = 2 * asin( sqrt(a) );

  float meters = EARTH_RADIUS_M * c;

  return meters;
}

// Returns the angle that is halfway between the specified headings
static float calc_mid_angle(float heading_1, float heading_2) {
  float h1_rad = DEG_TO_RAD(heading_1);
  float h2_rad = DEG_TO_RAD(heading_2);

  // avr-gcc compiles with gnu++1 and gnu11 standard; the trig functions accept
  // float params
  float resultant[2] = {0.0, 0.0};
  float resultant[0] = cos(h1_rad) + cos(h2_rad);
  float resultant[1] = sin(h1_rad) + sin(h2_rad);

  float mid_angle_rad = atan2(resultant[1], resultant[0]);
  float mid_angle_deg = RAD_TO_DEG(mid_angle_rad);

  if mid_angle_deg < 0.0 {
    mid_angle_deg += 360.0;
  }

  return mid_angle_deg;
}

static float calc_nav_heading(void) {

}

// Calculates a new position based on the current heading and distance
// traveled from the previous position
static GPS_Coordinate calc_position(GPS_Coordinate* prev_position, float distance, float heading) {
  float lat_rad = DEG_TO_RAD(prev_position.latitude);
  float long_rad = DEG_TO_RAD(prev_position.longitude);
  float heading_rad = DEG_TO_RAD(heading);

  float est_lat = 0.0;
  float est_long = 0.0;

  est_lat = asin( sin(lat_rad) *
    cos(distance / EARTH_RADIUS_M) +
    cos(lat_rad) *
    sin(distance / EARTH_RADIUS_M) *
    cos(heading_rad) );

  est_long = long_rad +
    atan2( sin(heading_rad) *
    sin(distance / EARTH_RADIUS_M) *
    cos(lat_rad),
    cos(distance / EARTH_RADIUS_M) -
    sin(lat_rad) *
    sin(est_lat) );

  GPS_Coordinate new_position;

  new_position.latitude = RAD_TO_DEG(est_lat);
  new_position.longitude = RAD_TO_DEG(est_long);

  return new_position;
}

// Calculates the relative bearing (i.e., the angle between the current
// heading and the waypoint bearing)
static float calc_relative_bearing(GPS_Coordinate* curr_position, GPS_Coordinate* way_position, float heading);

// Calculates the robot's current speed; result in meters per second
static float calc_speed(void);

// Gets the next waypoint
static GPS_Coordinate get_next_waypoint(void);

void update_all_inputs(void) {
  button_update();
  cmps10_update_all();
  gps_update();

  // TODO This function may need to be wrapped in another function which also
  // handles timestamp updates (for distance and speed calculations)
  odometer_update();

  return;
}

void update_all_nav(void) {
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
