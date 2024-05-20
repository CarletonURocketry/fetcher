/**
 * @file lsm6dso32.c
 * @brief Sensor API interface implementation for the LSM6DSO32 inertial module.
 *
 * Sensor API interface implementation for the LSM6DSO32 inertial module.
 *
 * See
 * https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
 * for the LSM6DSO32 data sheet.
 */
#include "lsm6dso32.h"
#include "sensor_api.h"
#include <errno.h>
#include <hw/i2c.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/** Acceleration due to gravity in m/s^2. */
#define GRAVIT_ACC 9.81f

/**
 * All units being converted are being stored in milli-units.
 *
 * Ex:
 * Milli-Gs per least significant bit to Gs conversion rate using 32g full scale range.
 * 32g FSR * 2 = 64g range * 1000 = 64000mg range
 * Result is given in 16b integer, so 65535 possible different values for 64000mg
 * Conversion factor = 64000/65535 (mg per bit)
 * See https://ozzmaker.com/accelerometer-to-g/
 * We will return units in regular units instead of milli-units, so the multiplication of 1000 is removed.
 */
#define MILLI_UNIT_PER_LSB_TO_UNIT 2.0f / 65535.0f

/** Type to store the context of the IMU. */
typedef struct {
    /** Acceleration full scale range. */
    uint8_t acc_fsr;
    /** Gyroscope full scale range. */
    uint16_t gyro_fsr;
} LSM6DSO32Context;

/** The different registers that are present in the IMU (and used by this program). */
enum imu_reg {
    WHO_AM_I = 0x0F,   /**< Returns the hard-coded address of the IMU on the I2C bus. */
    TIMESTAMP0 = 0x40, /**< First timestamp register (32 bits total) */
    STATUS_REG = 0x1E, /**< The status register of whether data is available. */
    CTRL1_XL = 0x10,   /**< Accelerometer control register 1 */
    CTRL2_G = 0x11,    /**< Gyroscope control register 2 */
    CTRL3_C = 0x12,    /**< Control register 3 */
    CTRL4_C = 0x13,    /**< Control register 4 */
    CTRL5_C = 0x14,    /**< Control register 5 */
    CTRL6_C = 0x15,    /**< Control register 6 */
    CTRL7_G = 0x16,    /**< Control register 7 */
    CTRL8_XL = 0x17,   /**< Control register 8 */
    CTRL9_XL = 0x18,   /**< Control register 9 */
    CTRL10_C = 0x19,   /**< Control register 10 */
    FIFO_CTRL4 = 0x0A, /**< The fourth FIFO control register (for setting continuous mode) */
    OUT_TEMP_L = 0x20, /**< The temperature data output low byte. (Two's complement) */
    OUT_TEMP_H = 0x21, /**< The temperature data output high byte. (Two's complement) */
    OUTX_L_G = 0x22,   /**< The angular rate sensor pitch axis (X) low byte. (Two's complement) */
    OUTX_H_G = 0x23,   /**< The angular rate sensor pitch axis (X) high byte. (Two's complement) */
    OUTY_L_G = 0x24,   /**< The angular rate sensor roll axis (Y) low byte. (Two's complement) */
    OUTY_H_G = 0x25,   /**< The angular rate sensor roll axis (Y) high byte. (Two's complement) */
    OUTZ_L_G = 0x26,   /**< The angular rate sensor yaw axis (Z) low byte. (Two's complement) */
    OUTZ_H_G = 0x27,   /**< The angular rate sensor yaw axis (Z) high byte. (Two's complement) */
    OUTX_L_A = 0x28,   /**< The linear acceleration (X) low byte. (Two's complement) */
    OUTX_H_A = 0x29,   /**< The linear acceleration (X) high byte. (Two's complement) */
    OUTY_L_A = 0x2A,   /**< The linear acceleration (Y) low byte. (Two's complement) */
    OUTY_H_A = 0x2B,   /**< The linear acceleration (Y) high byte. (Two's complement) */
    OUTZ_L_A = 0x2C,   /**< The linear acceleration (Z) low byte. (Two's complement) */
    OUTZ_H_A = 0x2D,   /**< The linear acceleration (Z) high byte. (Two's complement) */
    X_OFS_USR = 0x73,  /** The x-axis user offset correction for linear acceleration. */
    Y_OFS_USR = 0x74,  /** The y-axis user offset correction for linear acceleration. */
    Z_OFS_USR = 0x75,  /** The z-axis user offset correction for linear acceleration. */
};

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/**
 * Write data to a register of the LSM6DSO32.
 * WARNING: This function does not lock the I2C bus. It is meant to be called multiple times and therefore the bus must
 * be locked by the caller.
 * @param loc The location of the IMU on the I2C bus.
 * @param reg The address of the register to write to.
 * @param data The byte of data to write to the register.
 */
static errno_t lsm6dso32_write_byte(SensorLocation const *loc, const uint8_t reg, const uint8_t data) {

    // Construct header
    i2c_send_t header = {.slave = loc->addr, .stop = 1, .len = 2};

    // Create command
    uint8_t cmd[sizeof(header) + 2];
    memcpy(cmd, &header, sizeof(header));
    cmd[sizeof(header)] = reg;
    cmd[sizeof(header) + 1] = data;

    // Send command
    return devctl(loc->bus, DCMD_I2C_SEND, cmd, sizeof(cmd), NULL);
}

/**
 * Read data from register address `reg`.
 * WARNING: This function does not lock the I2C bus, as it is meant to be used in a continuous stream of calls.
 * @param loc The location of the IMU on the I2C bus.
 * @param reg The register address to read from.
 * @param buf The buffer to read data into.
 * @return EOK if the read was okay, otherwise the error status of the read command.
 */
static errno_t lsm6dso32_read_byte(SensorLocation const *loc, uint8_t reg, uint8_t *buf) {

    i2c_sendrecv_t read_hdr = {.stop = 1, .send_len = 1, .recv_len = 1, .slave = loc->addr};

    uint8_t read_cmd[sizeof(read_hdr) + 1];
    memcpy(read_cmd, &read_hdr, sizeof(read_hdr));
    read_cmd[sizeof(read_hdr)] = reg; // Data to be send is the register address

    errno_t err = devctl(loc->bus, DCMD_I2C_SENDRECV, read_cmd, sizeof(read_cmd), NULL);
    return_err(err);

    *buf = read_cmd[sizeof(read_hdr)]; // Received byte will be the last one
    return err;
}

/**
 * Reads temperature from the IMU.
 * @param loc The sensor location.
 * @temperature A pointer to where to store the temperature in degrees Celsius.
 * @return The error status of reading from the sensor. EOK if successful.
 */
int lsm6dso32_get_temp(SensorLocation const *loc, int16_t *temperature) {
    int err = lsm6dso32_read_byte(loc, OUT_TEMP_L, (uint8_t *)(temperature)); // Read low byte
    return_err(err);
    err = lsm6dso32_read_byte(loc, OUT_TEMP_H, (uint8_t *)(temperature) + 1); // Read high byte
    return_err(err);
    *temperature /= 256.0f + 25.0f; // In degrees Celsius
    return err;
}

/**
 * Read the linear acceleration data from the IMU.
 * @param x A pointer to where to store the X component of the acceleration in milli-Gs per LSB.
 * @param y A pointer to where to store the Y component of the acceleration in milli-Gs per LSB.
 * @param z A pointer to where to store the Z component of the acceleration in milli-Gs per LSB.
 * @return Any error that occurred while reading the sensor, EOK if successful.
 */
int lsm6dso32_get_accel(SensorLocation const *loc, int16_t *x, int16_t *y, int16_t *z) {

    int err = lsm6dso32_read_byte(loc, OUTX_L_A, (uint8_t *)(x)); // Read low byte
    return_err(err);
    err = lsm6dso32_read_byte(loc, OUTX_H_A, (uint8_t *)(x) + 1); // Read high byte
    return_err(err);

    err = lsm6dso32_read_byte(loc, OUTY_L_A, (uint8_t *)(y));
    return_err(err);
    err = lsm6dso32_read_byte(loc, OUTY_H_A, (uint8_t *)(y) + 1);
    return_err(err);

    err = lsm6dso32_read_byte(loc, OUTZ_L_A, (uint8_t *)(z));
    return_err(err);
    err = lsm6dso32_read_byte(loc, OUTZ_H_A, (uint8_t *)(z) + 1);
    return err;
}

/**
 * Read the linear angular velocity data from the IMU.
 * @param x A pointer to where to store the X component of the angular velocity in millidegrees per LSB.
 * @param y A pointer to where to store the Y component of the angular velocity in millidegrees per LSB.
 * @param z A pointer to where to store the Z component of the angular velocity in millidegrees per LSB.
 * @return Any error that occurred while reading the sensor, EOK if successful.
 */
int lsm6dso32_get_angular_vel(SensorLocation const *loc, int16_t *x, int16_t *y, int16_t *z) {
    int err = lsm6dso32_read_byte(loc, OUTX_L_G, (uint8_t *)(x)); // Read low byte
    return_err(err);
    err = lsm6dso32_read_byte(loc, OUTX_H_G, (uint8_t *)(x) + 1); // Read high byte
    return_err(err);

    err = lsm6dso32_read_byte(loc, OUTY_L_G, (uint8_t *)(y));
    return_err(err);
    err = lsm6dso32_read_byte(loc, OUTY_H_G, (uint8_t *)(y) + 1);
    return_err(err);

    err = lsm6dso32_read_byte(loc, OUTZ_L_G, (uint8_t *)(z));
    return_err(err);
    err = lsm6dso32_read_byte(loc, OUTZ_H_G, (uint8_t *)(z) + 1);
    return err;
}

/**
 * Converts acceleration in milli-Gs per LSB to meters per second squared. Results are stored back in the pointers
 * themselves. Passing NULL as any of the pointers will result in the calculation being skipped.
 * @param acc_fsr The full scale range of the accelerometer.
 * @param x A pointer to the X component of the acceleration in milli-Gs per LSB.
 * @param y A pointer to the Y component of the acceleration in milli-Gs per LSB.
 * @param z A pointer to the Z component of the acceleration in milli-Gs per LSB.
 */
void lsm6dso32_convert_accel(accel_fsr_e acc_fsr, int16_t *x, int16_t *y, int16_t *z) {
    float conversion_factor = acc_fsr * MILLI_UNIT_PER_LSB_TO_UNIT * GRAVIT_ACC;
    if (x) (*x) *= conversion_factor;
    if (y) (*y) *= conversion_factor;
    if (z) (*z) *= conversion_factor;
}

/**
 * Converts angular velocity in millidegrees per LSB to meters per second squared. Results are stored back in the
 * pointers themselves. Passing NULL as any of the pointers will result in the calculation being skipped.
 * @param gyro_fsr The full scale range of the gyroscope.
 * @param x A pointer to the X component of the angular velocity in millidegrees per LSB.
 * @param y A pointer to the Y component of the angular velocity in millidegrees per LSB.
 * @param z A pointer to the Z component of the angular velocity in millidegrees per LSB.
 */
void lsm6dso32_convert_angular_vel(gyro_fsr_e gyro_fsr, int16_t *x, int16_t *y, int16_t *z) {
    float conversion_factor = gyro_fsr * MILLI_UNIT_PER_LSB_TO_UNIT;
    if (x) (*x) *= conversion_factor;
    if (y) (*y) *= conversion_factor;
    if (z) (*z) *= conversion_factor;
}

/**
 * Performs a software reset of the LSM6DSO32.
 * @param loc The location of the IMU on the I2C bus.
 * @return Any error which occurred while resetting the IMU, EOK if successful.
 */
int lsm6dso32_reset(SensorLocation const *loc) { return lsm6dso32_write_byte(loc, CTRL3_C, 0x01); }

/**
 * Sets the accelerometer full scale range.
 * @param loc The location of the IMU on the I2C bus.
 * @param fsr The FSR to set.
 * @return Any error which occurred communicating with the IMU, EOK if successful, EINVAL if bad fsr.
 */
int lsm6dso32_set_acc_fsr(SensorLocation const *loc, accel_fsr_e fsr) {

    uint8_t reg_val;
    int err = lsm6dso32_read_byte(loc, CTRL1_XL, &reg_val); // Don't overwrite other configurations
    reg_val &= ~((1 << 3) | (1 << 2));                      // Clear FSR selection bits
    return_err(err);

    switch (fsr) {
    case LA_FS_4G:
        // All zeroes
        break;
    case LA_FS_8G:
        reg_val |= (0x2 << 3);
        break;
    case LA_FS_16G:
        reg_val |= (0x3 << 3);
        break;
    case LA_FS_32G:
        reg_val |= (0x1 << 3);
        break;
    default:
        return EINVAL;
    }
    return lsm6dso32_write_byte(loc, CTRL1_XL, reg_val);
}

/**
 * Sets the gyroscope full scale range.
 * @param loc The location of the IMU on the I2C bus.
 * @param fsr The FSR to set.
 * @return Any error which occurred communicating with the IMU, EOK if successful, EINVAL if bad fsr.
 */
int lsm6dso32_set_gyro_fsr(SensorLocation const *loc, gyro_fsr_e fsr) {

    uint8_t reg_val;
    int err = lsm6dso32_read_byte(loc, CTRL2_G, &reg_val); // Don't overwrite other configurations
    reg_val &= ~((1 << 3) | (1 << 2) | (1 << 1));          // Clear FSR selection bits
    return_err(err);

    switch (fsr) {
    case G_FS_125:
        reg_val |= (1 << 1);
        break;
    case G_FS_250:
        // All zeroes
        break;
    case G_FS_500:
        reg_val |= (1 << 2);
        break;
    case G_FS_1000:
        reg_val |= (1 << 3);
        break;
    case G_FS_2000:
        reg_val |= ((1 << 3) | (1 << 2));
        break;
    default:
        return EINVAL;
    }
    return lsm6dso32_write_byte(loc, CTRL2_G, reg_val);
}

/**
 * Sets the accelerometer output data rate.
 * NOTE: An ODR of 1.6Hz can only be set if high performance operating mode is disabled. Otherwise the effect will be
 * 12.6Hz ODR. It is the responsibility of the caller to set disable the high performance operating mode before or after
 * this function is called.
 * @param loc The location of the IMU on the I2C bus.
 * @param odr The ODR to set.
 * @return Any error which occurred communicating with the IMU, EOK if successful.
 */
int lsm6dso32_set_acc_odr(SensorLocation const *loc, accel_odr_e odr) {
    uint8_t reg_val;
    int err = lsm6dso32_read_byte(loc, CTRL1_XL, &reg_val); // Don't overwrite other configurations
    reg_val &= ~(0xF0);                                     // Clear ODR selection bits
    reg_val |= (uint8_t)(odr);                              // Set ODR selection
    return_err(err);
    return lsm6dso32_write_byte(loc, CTRL1_XL, reg_val);
}

/**
 * Sets the gyroscope output data rate.
 * @param loc The location of the IMU on the I2C bus.
 * @param odr The ODR to set.
 * @return Any error which occurred communicating with the IMU, EOK if successful.
 */
int lsm6dso32_set_gyro_odr(SensorLocation const *loc, gyro_odr_e odr) {
    uint8_t reg_val;
    int err = lsm6dso32_read_byte(loc, CTRL2_G, &reg_val); // Don't overwrite other configurations
    reg_val &= ~(0xF0);                                    // Clear ODR selection bits
    reg_val |= (uint8_t)(odr);                             // Set ODR selection
    return_err(err);
    return lsm6dso32_write_byte(loc, CTRL2_G, reg_val);
}

/**
 * Powers down the gyroscope.
 * @param loc The location of the IMU on the I2C bus.
 * @return Any error which occurred communicating with the IMU, EOK if successful.
 */
int lsm6dso32_disable_gyro(SensorLocation const *loc) { return lsm6dso32_set_gyro_odr(loc, 0); }

/**
 * Powers down the accelerometer.
 * @param loc The location of the IMU on the I2C bus.
 * @return Any error which occurred communicating with the IMU, EOK if successful.
 */
int lsm6dso32_disable_accel(SensorLocation const *loc) { return lsm6dso32_set_gyro_odr(loc, 0); }

/**
 * Prepares the LSM6DSO32 for reading.
 * @param sensor A reference to an LSM6DSO32 sensor.
 * @return The error status of the call. EOK if successful.
 */
static errno_t lsm6dso32_open(Sensor *sensor) {

    // Perform software reset
    errno_t err = lsm6dso32_write_byte(sensor, CTRL3_C, 0x01);
    return_err(err);

    // TODO: We will want to operate in continuous mode for our use case (polling)

    // Set the operating mode of the accelerometer to ODR of 6.66kHz (high perf)
    // TODO: set based on configured sensor performance
    // Full scale range of +-32g (necessary for rocket)
    ((LSM6DSO32Context *)(sensor->context.data))->acc_fsr = 32;
    err = lsm6dso32_write_byte(sensor, CTRL1_XL, 0xA4);
    return_err(err);

    // Set the operating mode of the gyroscope to ODR of 6.66kHz (high perf)
    // TODO: set based on configured sensor performance
    // TODO: what full-scale selection? Keep 500 for now
    ((LSM6DSO32Context *)(sensor->context.data))->gyro_fsr = 500;
    err = lsm6dso32_write_byte(sensor, CTRL2_G, 0xA1);
    return_err(err);

    // TODO: what filter configuration will give the best measurements?
    return err;
}
