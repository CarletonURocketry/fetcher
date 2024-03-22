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
#include <unistd.h>

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** A list of data types that can be read by the LSM6DSO32. */
static const SensorTag TAGS[] = {};

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
static errno_t lsm6ds032_open(Sensor *sensor) { return EOK; }

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
