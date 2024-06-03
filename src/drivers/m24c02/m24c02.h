#ifndef _M24C0X_H_
#define _M24C0X_H_

#include "../sensor_api.h"

/** The capacity of the M24C02 EEPROM in bytes. */
#define M24C02_CAP 256

/** The capacity of the M24C01 EEPROM in bytes. */
#define M24C01_CAP 128

int m24c0x_write_byte(SensorLocation const *loc, uint8_t addr, uint8_t data);
int m24c0x_write_page(SensorLocation const *loc, uint8_t addr, uint8_t const *data, size_t nbytes);

int m24c0x_read_cur_byte(SensorLocation const *loc, uint8_t *data);
int m24c0x_read_rand_byte(SensorLocation const *loc, uint8_t addr, uint8_t *data);
int m24c0x_seq_read_cur(SensorLocation const *loc, uint8_t *data, size_t nbytes);
int m24c0x_seq_read_rand(SensorLocation const *loc, uint8_t addr, uint8_t *data, size_t nbytes);
int m24c0x_erase(SensorLocation const *loc, size_t size);

#endif // _M24C0X_H_
