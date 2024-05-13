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
} sht41_prec_e;

/** The wattage to use for heating the SHT41 sensor. */
typedef enum {
    SHT41_WATT_200, /**< 200mW. */
    SHT41_WATT_110, /**< 110mW. */
    SHT41_WATT_20,  /**< 20mW. */
} sht41_wattage_e;

/** The duration to heat the SHT41 sensor for. */
typedef enum {
    SHT41_HEAT_1s,  /**< 1 second. */
    SHT41_HEAT_P1s, /**< 0.1 seconds. */
} sht41_dur_e;

errno_t sht41_reset(SensorLocation const *loc);
errno_t sht41_read(SensorLocation const *loc, sht41_prec_e precision, float *temperature, float *humidity);
errno_t sht41_serial_no(SensorLocation const *loc, uint32_t *serial_no);
errno_t sht41_heat(SensorLocation const *loc, sht41_dur_e duration, sht41_wattage_e wattage);

#endif // _SHT41_H_
