/*
 * file: sdcard.ino
 * created: 20160710
 * author(s): mr-augustine
 *
 * The sdcard Arduino file defines the SD card wrapper functions.
 */
#include <SD.h>
#include "kintobor.h"

#define SDCARD_CHIP_SELECT 53

uint8_t * data;
uint32_t data_size;
File data_file;

uint8_t sdcard_init(void * data_ptr, uint32_t size) {
  pinMode(SDCARD_CHIP_SELECT, OUTPUT);

  if (data_ptr != NULL) {
    data = (uint8_t *) data_ptr;
  } else {
    return 0;
  }

  if (size != 0) {
    data_size = size;
  }

  if (!SD.begin(SDCARD_CHIP_SELECT)) {
    uwrite_print_buff("SD Card didn't initialize\r\n");
    return 0;
  }

  if (!init_datafile()) {
    uwrite_print_buff("Could not start a new datafile\r\n");
    return 0;
  }

  return 1;
}

uint8_t init_datafile(void) {
  char filepath[32];

  // Ensure that a folder with the same name as the robot is created
  if (!SD.exists(ROBOT_NAME)) {
    SD.mkdir(ROBOT_NAME);
  }

  // Create the path to the new file
  // File names must be in the 8.3 format (i.e., 8 characters for the file name
  // and 3 characters for the file extension)
  uint16_t file_index;
  for (file_index = 1; file_index < UINT16_MAX; file_index++) {
    snprintf(filepath, sizeof(filepath), "/%s/k%05u.dat",
              ROBOT_NAME, file_index);

    if (!SD.exists(filepath)) {
      data_file = SD.open(filepath, FILE_WRITE);
      break;
    }
  }
  if (file_index == UINT16_MAX) {
    return 0;
  }

  if (!data_file) {
    return 0;
  }

  uwrite_print_buff(filepath);
  uwrite_print_buff(" was opened for write!\r\n");

  return 1;
}

void write_data(void) {
  if (data_file) {
    data_file.write(data, data_size);
  }

  return;
}

void sdcard_finish(void) {
  if (data_file) {
    data_file.close();
    uwrite_print_buff("File is closed\r\n");
  }

  return;
}
