/*
 * file: cmps11.h
 * created: 20160808
 * author(s): mr-augustine
 *
 * Lists the functions used to initialize and update the compass sensor.
 *
 * The extern "C" construct allows the main Arduino program to use the
 * functions declared below.
 */
#ifndef _CMPS11_H_
#define _CMPS11_H_

#define COMPASS_ADDR          0x60
#define COMPASS_HEADING_REG   2
#define COMPASS_PITCH_REG     4
#define COMPASS_ROLL_REG      5

#ifdef __cplusplus
extern "C" {
  uint8_t cmps11_init(void);

  void cmps11_update_all(void);
}
#endif // #ifdef __cplusplus

#endif // #ifndef _CMPS11_H_
