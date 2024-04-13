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
#include <errno.h>
#include <hw/i2c.h>
#include <stdbool.h>
// TODO: remove
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/** The different registers that are present in the IMU (and used by this program). */
typedef enum {
    WHO_AM_I = 0x0F,   /**< Returns the hard-coded address of the IMU on the I2C bus. */
    OUTX_L_A = 0x29,   /**< X-axis linear acceleration (16 bits) */
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
} IMUReg;

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** A list of data types that can be read by the LSM6DSO32. */
static const SensorTag TAGS[] = {};

/**
 * Write data to a register of the LSM6DS032.
 * @param sensor A pointer to a LSM6DS032 sensor instance.
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
 * Read `nbytes` from starting at register address `reg`.
 * WARNING: `buf` must contain enough space for `nbytes` bytes AND `sizeof(i2c_recv_t)`. Do not include the extra byte
 * count in nbytes.
 * @param sensor A reference to an LSM6DS032 sensor instance.
 * @param reg The register address to read from.
 * @param buf The buffer to read data into.
 * @param nbytes The number of bytes to read.
 */
static errno_t lsm6dso32_read_bytes(Sensor *sensor, uint8_t reg, void *buf, size_t nbytes) {

    // Construct command for sending register
    static i2c_send_t register_hdr = {.stop = 0, .slave = {0}, .len = 1};
    register_hdr.slave = sensor->loc.addr;

    // Send command to set register
    memcpy(buf, &register_hdr, sizeof(register_hdr));

    // Here I am using the buffer for receiving bytes as the storage for the send command, as it is gauranteed to at
    // least contain `sizeof(i2c_send_t) + 1` bytes, since `sizeof(i2c_send_t) == sizeof(i2c_recv_t)`, and there would
    // be no point in calling this function with `nbytes = 0`.
    ((uint8_t *)(buf))[sizeof(register_hdr)] = reg;
    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SEND, buf, sizeof(register_hdr) + 1, NULL);
    return_err(err);

    // Construct command to read
    static i2c_recv_t read_hdr = {.slave = {0}, .stop = 1, .len = 0};
    read_hdr.slave = sensor->loc.addr;
    read_hdr.len = nbytes;
    memcpy(buf, &read_hdr, sizeof(read_hdr));

    // Send command to read
    return devctl(sensor->loc.bus, DCMD_I2C_RECV, buf, sizeof(read_hdr) + nbytes, NULL);
}

/**
 * Reads the specified data from the LSM6DSO32.
 * @param sensor A reference to an LSM6DSO32 sensor.
 * @param tag The tag of the data type that should be read.
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes that were written into the byte array buffer.
 * @return Error status of reading from the sensor. EOK if successful.
 */
static errno_t lsm6ds032_read(Sensor *sensor, const SensorTag tag, void *buf, size_t *nbytes) { return EOK; }

/**
 * Prepares the LSM6DS032 for reading.
 * @param sensor A reference to an LSM6DS032 sensor.
 * @return The error status of the call. EOK if successful.
 */
static errno_t lsm6ds032_open(Sensor *sensor) {

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

    uint8_t result[sizeof(i2c_send_t) + 1];
    err = lsm6dso32_read_bytes(sensor, STATUS_REG, result, 1);

    printf("%08x\n", (uint32_t)result[sizeof(i2c_send_t)]);
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
    sensor->open = &lsm6ds032_open;
    sensor->read = &lsm6ds032_read;
}
