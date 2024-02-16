/**
 * @file sht41.h
 * @brief Sensor API implementation for the SHT41 temperature and humidity sensor.
 */
#include "sht41.h"
#include "hw/i2c.h"
#include "sensor_api.h"

/** A list of data types that can be read by the SHT41 */
static const SensorTag TAGS[] = {TAG_TEMPERATURE, TAG_HUMIDITY};

/** Some of the I2C commands that can be used on the SHT41 sensor. */
typedef enum sht41_cmd_t {
    CMD_RESET = 0x94,          /**< Soft reset command */
    CMD_SERIAL_NUM = 0x89,     /**< Read serial number command */
    CMD_READ_LOW_PREC = 0xE0,  /**< Low precision read temp & humidity command*/
    CMD_READ_MED_PREC = 0xF6,  /**< Med precision read temp & humidity command*/
    CMD_READ_HIGH_PREC = 0xFD, /**< High precision read temp & humidity command*/
} SHT41Cmd;

/**
 * Reads the specified data from the SHT41.
 * @param sensor A reference to an SHT41 sensor.
 * @param tag The tag of the data type that should be read.
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes that were written into the byte array buffer.
 * @return Error status of reading from the sensor. EOK if successful.
 */
static errno_t sht41_read(Sensor *sensor, const SensorTag tag, void *buf, uint8_t *nbytes) {}

/**
 * Prepares the SHT41 for reading and initializes the sensor context with the calibration coefficients.
 * @param sensor A reference to an SHT41 sensor.
 * @return The error status of the call. EOK if successful.
 */
static errno_t sht41_open(Sensor *sensor) {}

/**
 * Initializes a sensor struct with the interface to interact with the SHT41.
 * @param sensor The sensor interface to be initialized.
 * @param bus The file descriptor of the I2C bus.
 * @param addr The address of the sensor on the I2C bus.
 * @param precision The precision to read measurements with.
 */
void sht41_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision) {
    sensor->precision = precision;
    sensor->loc = (SensorLocation){.bus = bus, .addr = {.addr = (addr & 0x7F), .fmt = I2C_ADDRFMT_7BIT}};
    // sensor->tag_list = (SensorTagList){.tags = } Set the tags later
    // sensor->max_dsize = set later
    // sensor->context.size = calculate later
    sensor->open = &sht41_open;
    sensor->read = &sht41_read;
}
