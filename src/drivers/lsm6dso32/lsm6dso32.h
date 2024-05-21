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

/** The value that should be returned from the WHOAMI register. */
#define WHOAMI_VALUE 0x6C

/** Represents the possible full scale range settings for linear acceleration in Gs (gravitational acceleration). */
typedef enum {
    LA_FS_4G = 4,   /**< Full scale range of +/- 4Gs */
    LA_FS_8G = 8,   /**< Full scale range of +/- 8Gs */
    LA_FS_16G = 16, /**< Full scale range of +/- 16Gs */
    LA_FS_32G = 32, /**< Full scale range of +/- 32Gs */
} accel_fsr_e;

/** Represents the possible full scale range settings for the gyroscope in degrees per second. */
typedef enum {
    G_FS_125 = 125,   /**< Full scale range of +/-125 degrees per second. */
    G_FS_250 = 250,   /**< Full scale range of +/-250 degrees per second. */
    G_FS_500 = 500,   /**< Full scale range of +/-500 degrees per second. */
    G_FS_1000 = 1000, /**< Full scale range of +/-1000 degrees per second. */
    G_FS_2000 = 2000, /**< Full scale range of +/-2000 degrees per second. */
} gyro_fsr_e;

/** Possible output data rates for linear acceleration in Hz. */
typedef enum {
    LA_ODR_1_6 = 0xB0,  /** 1.6 Hz */
    LA_ODR_12_5 = 0x10, /** 12.5 Hz */
    LA_ODR_26 = 0x20,   /** 26 Hz */
    LA_ODR_52 = 0x30,   /** 52 Hz */
    LA_ODR_104 = 0x40,  /** 104 Hz */
    LA_ODR_208 = 0x50,  /** 208 Hz */
    LA_ODR_416 = 0x60,  /** 416 Hz */
    LA_ODR_833 = 0x70,  /** 833 Hz */
    LA_ODR_1666 = 0x80, /** 1666 Hz */
    LA_ODR_3332 = 0x90, /** 3332 Hz */
    LA_ODR_6664 = 0xA0, /** 6644 Hz */
} accel_odr_e;

/** Possible output data rates for the gyroscope in Hz. */
typedef enum {
    G_ODR_12_5 = 0x10, /** 12.5 Hz */
    G_ODR_26 = 0x20,   /** 26 Hz */
    G_ODR_52 = 0x30,   /** 52 Hz */
    G_ODR_104 = 0x40,  /** 104 Hz */
    G_ODR_208 = 0x50,  /** 208 Hz */
    G_ODR_416 = 0x60,  /** 416 Hz */
    G_ODR_833 = 0x70,  /** 833 Hz */
    G_ODR_1666 = 0x80, /** 1666 Hz */
    G_ODR_3332 = 0x90, /** 3332 Hz */
    G_ODR_6664 = 0xA0, /** 6644 Hz */
} gyro_odr_e;

int lsm6dso32_reset(SensorLocation const *loc);
int lsm6dso32_mem_reboot(SensorLocation const *loc);
int lsm6dso32_disable_accel(SensorLocation const *loc);
int lsm6dso32_disable_gyro(SensorLocation const *loc);

int lsm6dso32_set_acc_fsr(SensorLocation const *loc, accel_fsr_e fsr);
int lsm6dso32_set_gyro_fsr(SensorLocation const *loc, gyro_fsr_e fsr);
int lsm6dso32_set_acc_odr(SensorLocation const *loc, accel_odr_e odr);
int lsm6dso32_set_gyro_odr(SensorLocation const *loc, gyro_odr_e odr);

int lsm6dso32_get_temp(SensorLocation const *loc, int16_t *temperature);
int lsm6dso32_get_accel(SensorLocation const *loc, int16_t *x, int16_t *y, int16_t *z);
int lsm6dso32_get_angular_vel(SensorLocation const *loc, int16_t *x, int16_t *y, int16_t *z);
int lsm6dso32_whoami(SensorLocation const *loc, uint8_t *val);

void lsm6dso32_convert_accel(accel_fsr_e acc_fsr, int16_t *x, int16_t *y, int16_t *z);
void lsm6dso32_convert_angular_vel(gyro_fsr_e gyro_fsr, int16_t *x, int16_t *y, int16_t *z);

#endif // _LSM6DSO32_
