/**
 * @file sht41.h
 * @brief Header file containing the necessary types and function prototypes for controlling the SHT41 temperature and
 * humidity sensor
 *
 * Define SHT41_USE_CRC_LOOKUP to use a lookup table for the sht41's CRC. CRCs on the SHT41 disabled otherwise.
 */
#ifndef _SHT41_H_
#define _SHT41_H_

#include "../sensor_api.h"
#include <stdint.h>

/** The precision to read the SHT41 sensor with. */
typedef enum {
    SHT41_LOW_PRES,  /**< Low precision. */
    SHT41_MED_PRES,  /**< Medium precision. */
    SHT41_HIGH_PRES, /**< High precision. */
} SHT41Precision;

errno_t sht41_reset(SensorLocation *loc);
errno_t sht41_read(SensorLocation *loc, SHT41Precision precision, float *temperature, float *humidity);
errno_t sht41_serial_no(SensorLocation *loc, uint32_t *serial_no);

#endif // _SHT41_H_
