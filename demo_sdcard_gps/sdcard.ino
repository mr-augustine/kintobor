/*
 * file: sdcard.ino
 * created: 20160611
 * author(s): mr-augustine
 *
 * The sdcard Arduino file defines the SD card wrapper functions.
 */
#include <SD.h>

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
    Serial.println("SD Card didn't initialize");
    return 0;
  }

  if (!init_datafile()) {
    Serial.println("Could not start a new datafile");
    return 0;
  }

  return 1;
}

uint8_t init_datafile(void) {
  if (!SD.exists("/today")) {
    SD.mkdir("/today");
  }
  // File names must be in the 8.3 format (i.e., 8 characters for the file name
  // and 3 characters for the file extension)
  data_file = SD.open("today/kintobor.dat", FILE_WRITE);

  if (!data_file) {
    return 0;
  }

  Serial.println("Data file was opened for write!");
  return 1;
}

void write_data(void) {
  if (data_file) {
    data_file.write(data, data_size);
  }
}

void sdcard_finish(void) {
  if (data_file) {
    data_file.close();
    Serial.println("File is closed");
  }
}
