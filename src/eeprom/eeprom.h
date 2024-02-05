#ifndef _M24C02_EEPROM_H
#define _M24C02_EEPROM_H

#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

errno_t eeprom_read(uint8_t addr, int bus, void *buf, size_t n);

#endif // _M24C02_EEPROM_H
