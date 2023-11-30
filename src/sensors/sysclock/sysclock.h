#ifndef _SYS_CLOCK_H_
#define _SYS_CLOCK_H_

#include "../sensor_api.h"
#include <stdint.h>

void sysclock_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision);

#endif // _SYS_CLOCK_H_
