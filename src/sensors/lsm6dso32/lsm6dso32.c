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
#include <stdio.h>
#include <errno.h>
#include <hw/i2c.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

/** The different registers that are present in the IMU (and used by this program). */
typedef enum {
    WHO_AM_I = 0x0F,
} IMUReg;

/** Macro to create the read address of the IMU. */
#define read_addr(addr) (addr | 0x01)

/** Macro to create the write address of the IMU. */
#define write_addr(addr) (addr & 0xFE)

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** A list of data types that can be read by the LSM6DSO32. */
static const SensorTag TAGS[] = {};

static errno_t lsm6dso32_write(Sensor const *sensor, void *buf, size_t nbytes) {

    // Construct header
    i2c_send_t header = {
        .slave = sensor->loc.addr,
        .stop = 1,
        .len = nbytes,
    };
    header.slave.addr = write_addr(header.slave.addr);

    // Create command
    uint8_t cmd[nbytes + sizeof(header)];
    memcpy(cmd, &header, sizeof(header));
    memcpy(&cmd[sizeof(header)], buf, nbytes);

    // Send command
    return devctl(sensor->loc.bus, DCMD_I2C_SEND, cmd, nbytes + sizeof(header), NULL);
}

/**
 * Read `nbytes` from starting at register address `reg`.
 * WARNING: `buf` must contain enough space for `nbytes` bytes AND `sizeof(i2c_recv_t)`. Do not include the extra byte
 * count in nbytes.
 */
static errno_t lsm6dso32_read_bytes(Sensor *sensor, uint8_t reg, void *buf, size_t nbytes) {

    // Construct command for sending register
    i2c_send_t register_hdr = {
        .stop = 0,
        .slave = sensor->loc.addr,
        .len = 1,
    };
    register_hdr.slave.addr = read_addr(register_hdr.slave.addr);

    // Send command to set register
    memcpy(buf, &register_hdr, sizeof(register_hdr));

    // Here I am using the buffer for receiving bytes as the storage for the send command, as it is gauranteed to at
    // least contain `sizeof(i2c_send_t) + 1` bytes, since `sizeof(i2c_send_t) == sizeof(i2c_recv_t)`, and there would
    // be no point in calling this function with `nbytes = 0`.
    ((uint8_t *)(buf))[sizeof(register_hdr)] = reg;
    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SEND, buf, sizeof(register_hdr) + 1, NULL);
    return_err(err);

    // Construct command to read
    i2c_recv_t read_hdr = {
        .slave = sensor->loc.addr,
        .stop = 1,
        .len = nbytes,
    };
    read_hdr.slave.addr = read_addr(read_hdr.slave.addr);
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

    uint8_t result[sizeof(i2c_send_t) + 1];
    errno_t err = lsm6dso32_read_bytes(sensor, WHO_AM_I, result, 1);

    printf("%02x\n", result[sizeof(i2c_send_t)]);
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
