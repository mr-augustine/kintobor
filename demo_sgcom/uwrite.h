/*
 * file: uwrite.h
 * created: 20160807
 * author(s): mr-augustine
 *
 * Lists the functions used to write text and values to the serial port.
 * This library was implemented to help create debug print statements.
 *
 * The extern "C" construct allows the main Arduino program to use the
 * functions declared below.
 */
#ifndef _UWRITE_H_
#define _UWRITE_H_

#define BUFF_SIZE 16

#ifdef __cplusplus
extern "C" {
  uint8_t uwrite_init(void);
  void uwrite_print_buff(char * char_buff);
  void uwrite_println_byte(void * a_byte);
  void uwrite_println_short(void * a_short);
  void uwrite_println_long(void * a_long);
}
#endif // #ifdef __cplusplus

#endif // #ifndef _UWRITE_H_
