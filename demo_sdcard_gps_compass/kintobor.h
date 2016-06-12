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
#include "statevars.h"
#include "uwrite.h"

#define ROBOT_NAME ("kintobor")

#ifdef __cplusplus
extern "C" {
  void update_all_inputs(void);
}
#endif // #ifdef __cplusplus

#endif // #ifndef _KINTOBOR_H_
