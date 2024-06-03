#ifndef _M24C02_H_
#define _M24C02_H_

#include "../sensor_api.h"

/** The capacity of the M24C02 EEPROM in bytes. */
#define M24C02_CAP 256

int m24c02_write_byte(SensorLocation const *loc, uint8_t addr, uint8_t data);
int m24c02_write_page(SensorLocation const *loc, uint8_t addr, uint8_t const *data, size_t nbytes);

int m24c02_read_cur_byte(SensorLocation const *loc, uint8_t *data);
int m24c02_read_rand_byte(SensorLocation const *loc, uint8_t addr, uint8_t *data);
int m24c02_seq_read_cur(SensorLocation const *loc, uint8_t *data, size_t nbytes);
int m24c02_seq_read_rand(SensorLocation const *loc, uint8_t addr, uint8_t *data, size_t nbytes);

#endif // _M24C02_H_
