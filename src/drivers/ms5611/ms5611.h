/**
 * @file ms5611.h
 * @brief Header file containing the necessary types and function prototypes for controlling the MS611 Barometric
 * Pressure Sensor.
 *
 * Header file containing the necessary types and function prototypes for controlling the MS611 Barometric Pressure
 * Sensor.
 */
#ifndef _MS5611_H_
#define _MS5611_H_

#include "../sensor_api.h"
#include <stdbool.h>
#include <stdint.h>

#define MS5611_COEFFICIENT_COUNT 8

/** The resolution that is used to read the data of the MS5611 */
typedef enum ms5611_resolution_t {
    ADC_RES_256 = 0x00,  /**< ADC OSR=256 */
    ADC_RES_512 = 0x02,  /**< ADC OSR=512 */
    ADC_RES_1024 = 0x04, /**< ADC OSR=1024 */
    ADC_RES_2048 = 0x06, /**< ADC OSR=2048 */
    ADC_RES_4096 = 0x08, /**< ADC OSR=4096 */
} MS5611Resolution;

/** Contains information needed for the MS5611 between calls. */
typedef struct {
    uint16_t coefs[MS5611_COEFFICIENT_COUNT]; /**< The calibration coefficients of the sensor. */
    double ground_pressure;                   /**< The pressure at the ground level; set when the sensor is started. */
} MS5611Context;

errno_t ms5611_reset(SensorLocation *loc);
errno_t ms5611_read_all(SensorLocation *loc, MS5611Resolution res, MS5611Context *ctx, bool precise, double *temp,
                        double *pressure, double *altitude);
errno_t ms5611_init_coefs(SensorLocation *loc, MS5611Context *ctx);

#endif // _MS5611_H_
