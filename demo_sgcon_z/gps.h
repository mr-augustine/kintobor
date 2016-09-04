/*
 * file: gps_bn.h
 * created: 20160902
 * author(s): mr-augustine
 *
 * Lists the functions used to initialize and update the GPS sensor.
 *
 * The extern "C" construct allows the main Arduino program to use the
 * functions declared below.
 */
#ifndef _GPS_BN_H_
#define _GPS_BN_H_

#define GPGGA_START             "$GPGGA"
#define GPGSA_START             "$GPGSA"
#define GPRMC_START             "$GPRMC"
#define GPVTG_START             "$GPVTG"
#define START_LENGTH            6
#define GPS_CHECKSUM_LENGTH     2
#define GPS_INVALID_HEX_CHAR    0xFF
#define GPS_FIELD_BUFF_SZ       8
#define GPS_NO_FIX              '0'
#define GPS_FIX_AVAIL           '1'
#define GPS_DIFF_FIX_AVAIL      '2'
#define GPS_TIME_WIDTH          6
#define GPS_SENTENCE_BUFF_SZ    128
#define GPS_SENTENCE_END        '\n'
#define GPS_SENTENCE_START      '$'
#define LAT_LONG_FIELD_LENGTH   9
#define NUM_GPS_SENTENCE_BUFFS  4

#ifdef __cplusplus
extern "C" {
  uint8_t gps_init(void);

  void gps_update(void);
}
#endif // #ifdef __cplusplus

#endif // #ifndef _GPS_BN_H_
