/**
 * @file sht41.h
 * @brief Header file containing the necessary types and function prototypes for controlling the SHT41 temperature and
 * humidity sensor
 */
#ifndef _SHT41_H_
#define _SHT41_H_

#include "../sensor_api.h"
#include <stdint.h>

// Define SHT41_USE_CRC_LOOKUP to use a lookup table for the sht41's CRC. CRCs on the SHT41 disabled otherwise

void sht41_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision);

#endif // _SHT41_H_
