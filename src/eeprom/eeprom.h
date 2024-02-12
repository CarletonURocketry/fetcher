#ifndef _M24C02_EEPROM_H
#define _M24C02_EEPROM_H

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

/** The maximum capacity of the EEPROM in bytes. */
#define EEPROM_CAP 128

errno_t eeprom_read(uint8_t addr, int bus, void *buf, size_t n);
const uint8_t *eeprom_contents(int bus);

#endif // _M24C02_EEPROM_H
