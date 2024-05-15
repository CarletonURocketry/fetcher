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

/** A list of data types that can be read by the LSM6DSO32. */
static const SensorTag TAGS[] = {TAG_TEMPERATURE, TAG_LINEAR_ACCEL, TAG_ANGULAR_VEL};

/**
 * Write data to a register of the LSM6DSO32.
 * WARNING: This function does not lock the I2C bus. It is meant to be called multiple times and therefore the bus must
 * be locked by the caller.
 * @param sensor A pointer to a LSM6DSO32 sensor instance.
 * @param reg The address of the register to write to.
 * @param data The byte of data to write to the register.
 */
static errno_t lsm6dso32_write_byte(Sensor const *sensor, const uint8_t reg, const uint8_t data) {

    // Construct header
    i2c_send_t header = {.slave = {0}, .stop = 1, .len = 2};
    header.slave = sensor->loc.addr;

    // Create command
    uint8_t cmd[sizeof(header) + 2];
    memcpy(cmd, &header, sizeof(header));
    cmd[sizeof(header)] = reg;
    cmd[sizeof(header) + 1] = data;

    // Send command
    return devctl(sensor->loc.bus, DCMD_I2C_SEND, cmd, sizeof(cmd), NULL);
}

/**
 * Read data from register address `reg`.
 * WARNING: This function does not lock the I2C bus, as it is meant to be used in a continuous stream of calls.
 * @param sensor A reference to an LSM6DSO32 sensor instance.
 * @param reg The register address to read from.
 * @param buf The buffer to read data into.
 * @return EOK if the read was okay, otherwise the error status of the read command.
 */
static errno_t lsm6dso32_read_byte(Sensor *sensor, uint8_t reg, uint8_t *buf) {

    i2c_sendrecv_t read_hdr = {.stop = 1, .send_len = 1, .recv_len = 1, .slave = {0}};
    read_hdr.slave = sensor->loc.addr;

    uint8_t read_cmd[sizeof(read_hdr) + 1];
    memcpy(read_cmd, &read_hdr, sizeof(read_hdr));
    read_cmd[sizeof(read_hdr)] = reg; // Data to be send is the register address

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SENDRECV, read_cmd, sizeof(read_cmd), NULL);
    if (err != EOK) return err;

    *buf = read_cmd[sizeof(read_hdr)]; // Received byte will be the last one
    return err;
}

/**
 * Reads the specified data from the LSM6DSO32.
 * @param sensor A reference to an LSM6DSO32 sensor.
 * @param tag The tag of the data type that should be read.
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes that were written into the byte array buffer.
 * @return Error status of reading from the sensor. EOK if successful. EINVAL if unrecognized tag.
 */
static errno_t lsm6dso32_read(Sensor *sensor, const SensorTag tag, void *buf, size_t *nbytes) {
    errno_t err;

    switch (tag) {
    case TAG_TEMPERATURE: {
        int16_t temp;
        err = lsm6dso32_read_byte(sensor, OUT_TEMP_L, (uint8_t *)(&temp)); // Read low byte
        return_err(err);
        err = lsm6dso32_read_byte(sensor, OUT_TEMP_H, (uint8_t *)(&temp) + 1); // Read high byte
        return_err(err);

        *(float *)buf = (float)(temp) / 256.0f + 25.0f; // In degrees Celsius
        *nbytes = sizeof(float);
        break;
    }
    case TAG_LINEAR_ACCEL: {

        int16_t x;
        err = lsm6dso32_read_byte(sensor, OUTX_L_A, (uint8_t *)(&x)); // Read low byte
        return_err(err);
        err = lsm6dso32_read_byte(sensor, OUTX_H_A, (uint8_t *)(&x) + 1); // Read high byte
        return_err(err);

        int16_t y;
        err = lsm6dso32_read_byte(sensor, OUTY_L_A, (uint8_t *)(&y));
        return_err(err);
        err = lsm6dso32_read_byte(sensor, OUTY_H_A, (uint8_t *)(&y) + 1);
        return_err(err);

        int16_t z;
        err = lsm6dso32_read_byte(sensor, OUTZ_L_A, (uint8_t *)(&z));
        return_err(err);
        err = lsm6dso32_read_byte(sensor, OUTZ_H_A, (uint8_t *)(&z) + 1);
        return_err(err);

        // Converts milli-Gs per LSB to m/s^2
        float conversion_factor =
            (float)((LSM6DSO32Context *)(sensor->context.data))->acc_fsr * MILLI_UNIT_PER_LSB_TO_UNIT * GRAVIT_ACC;

        ((vec3d_t *)(buf))->x = (float)(x)*conversion_factor;
        ((vec3d_t *)(buf))->y = (float)(y)*conversion_factor;
        ((vec3d_t *)(buf))->z = (float)(z)*conversion_factor;
        *nbytes = sizeof(vec3d_t);
        break;
    }
    case TAG_ANGULAR_VEL: {

        int16_t x;
        err = lsm6dso32_read_byte(sensor, OUTX_L_G, (uint8_t *)(&x)); // Read low byte
        return_err(err);
        err = lsm6dso32_read_byte(sensor, OUTX_H_G, (uint8_t *)(&x) + 1); // Read high byte
        return_err(err);

        int16_t y;
        err = lsm6dso32_read_byte(sensor, OUTY_L_G, (uint8_t *)(&y));
        return_err(err);
        err = lsm6dso32_read_byte(sensor, OUTY_H_G, (uint8_t *)(&y) + 1);
        return_err(err);

        int16_t z;
        err = lsm6dso32_read_byte(sensor, OUTZ_L_G, (uint8_t *)(&z));
        return_err(err);
        err = lsm6dso32_read_byte(sensor, OUTZ_H_G, (uint8_t *)(&z) + 1);
        return_err(err);

        // Converts millidegrees per second per LSB to degrees per second
        float conversion_factor =
            (float)((LSM6DSO32Context *)(sensor->context.data))->gyro_fsr * MILLI_UNIT_PER_LSB_TO_UNIT;

        ((vec3d_t *)(buf))->x = (float)(x)*conversion_factor;
        ((vec3d_t *)(buf))->y = (float)(y)*conversion_factor;
        ((vec3d_t *)(buf))->z = (float)(z)*conversion_factor;
        *nbytes = sizeof(vec3d_t);
        break;
    }
    default:
        return EINVAL;
    }

    return err;
}

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

/**
 * Initializes a sensor struct with the interface to interact with the LSM6DSO32.
 * @param sensor The sensor interface to be initialized.
 * @param bus The file descriptor of the I2C bus.
 * @param addr The address of the sensor on the I2C bus.
 * @param precision The precision to read measurements with.
 */
void lsm6dso32_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision) {
    sensor->precision = precision;
    sensor->loc = (SensorLocation){.bus = bus, .addr = {.addr = (addr & 0x7F), .fmt = I2C_ADDRFMT_7BIT}};
    sensor->tag_list = (SensorTagList){.tags = TAGS, .len = sizeof(TAGS) / sizeof(SensorTag)};
    sensor->context.size = sizeof(LSM6DSO32Context);
    sensor->open = &lsm6dso32_open;
    sensor->read = &lsm6dso32_read;
}
