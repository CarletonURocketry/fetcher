/**
 * @file ms5611.h
 * @brief Header file containing the necessary types and function prototypes for controlling the MS611 Barometric
 * Pressure Sensor.
 */
#ifndef _MS5611_H_
#define _MS5611_H_

#include "../sensor_api.h"
#include <stdint.h>

void ms5611_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision);

#endif // _MS5611_H_
