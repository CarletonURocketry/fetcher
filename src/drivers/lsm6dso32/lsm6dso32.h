/**
 * @file lsm6dso32.h
 * @brief Header file containing the necessary types and function prototypes for controlling the LSM6DSO32 inertial
 * module.
 *
 * Header file containing the necessary types and function prototypes for controlling the LSM6DSO32 inertial
 * module.
 */
#ifndef _LSM6DSO32_
#define _LSM6DSO32_

#include "../sensor_api.h"
#include <stdint.h>

void lsm6dso32_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision);

#endif // _LSM6DSO32_
