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
// TODO: remove
#include <stdio.h>
#include <string.h>
#include <unistd.h>

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
};

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** A list of data types that can be read by the LSM6DSO32. */
static const SensorTag TAGS[] = {TAG_TEMPERATURE, TAG_LINEAR_ACCEL, TAG_ANGULAR_ACCEL};

/**
 * Write data to a register of the LSM6DSO32.
 * @param sensor A pointer to a LSM6DSO32 sensor instance.
 * @param reg The address of the register to write to.
 * @param data The byte of data to write to the register.
 */
static errno_t lsm6dso32_write_byte(Sensor const *sensor, const uint8_t reg, const uint8_t data) {

    // Construct header
    static i2c_send_t header = {.slave = {0}, .stop = 1, .len = 2};
    header.slave = sensor->loc.addr;

    // Create command
    static uint8_t cmd[sizeof(header) + 2];
    memcpy(cmd, &header, sizeof(header));
    cmd[sizeof(header)] = reg;
    cmd[sizeof(header) + 1] = data;

    // Send command
    return devctl(sensor->loc.bus, DCMD_I2C_SEND, cmd, sizeof(cmd), NULL);
}

/**
 * Read data from register address `reg`.
 * @param sensor A reference to an LSM6DSO32 sensor instance.
 * @param reg The register address to read from.
 * @param buf The buffer to read data into.
 * @return EOK if the read was okay, otherwise the error status of the read command.
 */
static errno_t lsm6dso32_read_byte(Sensor *sensor, uint8_t reg, uint8_t *buf) {

    static i2c_sendrecv_t read_hdr = {.stop = 1, .send_len = 1, .recv_len = 1, .slave = {0}};
    read_hdr.slave = sensor->loc.addr;

    static uint8_t read_cmd[sizeof(read_hdr) + 1];
    memcpy(read_cmd, &read_hdr, sizeof(read_hdr));
    read_cmd[sizeof(read_hdr)] = reg; // Data to be send is the register address

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SENDRECV, read_cmd, sizeof(read_cmd), NULL);
    return_err(err);

    *buf = read_cmd[sizeof(read_hdr)]; // Received byte will be the last one
    return EOK;
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
    static errno_t err;

    switch (tag) {
    case TAG_TEMPERATURE: {
        int16_t temp = 0;
        err = lsm6dso32_read_byte(sensor, OUT_TEMP_L, (uint8_t *)(&temp)); // Read low byte
        return_err(err);
        err = lsm6dso32_read_byte(sensor, OUT_TEMP_H, (uint8_t *)(&temp) + 1); // Read high byte
        return_err(err);
        *(float *)buf = (float)(temp) / 256.0f + 25.0f; // In degrees celsius
        *nbytes = sizeof(float);
        break;
    }
    case TAG_LINEAR_ACCEL:
        break;
    case TAG_ANGULAR_ACCEL:
        break;
    default:
        return EINVAL;
    }
    return EOK;
}

/**
 * Prepares the LSM6DSO32 for reading.
 * @param sensor A reference to an LSM6DSO32 sensor.
 * @return The error status of the call. EOK if successful.
 */
static errno_t lsm6dso32_open(Sensor *sensor) {

    // TODO: We will want to operate in continuous mode for our use case (polling)

    // Set the operating mode of the accelerometer to ODR of 6.66kHz (high perf)
    // TODO: set based on configured sensor performance
    // Full scale rang of +-32g (necessary for rocket)
    errno_t err = lsm6dso32_write_byte(sensor, CTRL1_XL, 0xA4);
    return_err(err);

    // Set the operating mode of the gyroscope to ODR of 6.66kHz (high perf)
    // TODO: set based on configured sensor performance
    // TODO: what full-scale selection?
    err = lsm6dso32_write_byte(sensor, CTRL2_G, 0xA0);
    return_err(err);

    size_t nbytes;
    float temp;
    lsm6dso32_read(sensor, TAG_TEMPERATURE, &temp, &nbytes);
    printf("%f\n", temp);

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
    sensor->context.size = 0;
    sensor->open = &lsm6dso32_open;
    sensor->read = &lsm6dso32_read;
}
