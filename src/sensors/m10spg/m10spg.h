/**
 * @file m10spg.h
 * @brief Header file containing the necessary types and function prototypes for controlling the MAX-M10S gps sensor.
 *
 * Header file containing the necessary types and function prototypes for controlling the MAX-M10S gps sensor.
 */

#ifndef _MAXM10S_
#define _MAXM10S_

#include "../sensor_api.h"
#include <stdint.h>

void m10spg_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision);

#endif // _MAXM10S_
