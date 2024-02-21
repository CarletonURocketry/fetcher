/**
 * @file sht41.h
 * @brief Sensor API implementation for the SHT41 temperature and humidity sensor.
 */
#include "sht41.h"
#include "../sensor_api.h"
#include "hw/i2c.h"
#include <math.h>
#include <string.h>
#include <unistd.h>

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** Defines position of most significant byte in the sensor's response for temperatue */
#define SHT41_T_MSB 0

/** Defines position of the least significant byte in the sensor's response for temperature  */
#define SHT41_T_LSB 1

/** Defines position of the most significant byte in the sensor's response for humidity */
#define SHT41_RH_MSB 3

/** Defines position of the least significant byte in the sensor's reponse for humidity */
#define SHT41_RH_LSB 4

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

/** Implementation of precision in sensor API for read precision. */
static const SHT41Cmd PRECISIONS[] = {
    [PRECISION_LOW] = CMD_READ_LOW_PREC,
    [PRECISION_MED] = CMD_READ_MED_PREC,
    [PRECISION_HIGH] = CMD_READ_HIGH_PREC,
};

/** Meausrement times for the various precisions, in microseconds */
static const uint16_t MEASUREMENT_TIMES[] = {
    [PRECISION_LOW] = 1600,
    [PRECISION_MED] = 4500,
    [PRECISION_HIGH] = 8300,
};

/**
 * Reads the specified data from the SHT41.
 * @param sensor A reference to an SHT41 sensor.
 * @param tag The tag of the data type that should be read.
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes that were written into the byte array buffer.
 * @return Error status of reading from the sensor. EOK if successful.
 */
static errno_t sht41_read(Sensor *sensor, const SensorTag tag, void *buf, size_t *nbytes) {
    // No point calling a separate function, since the reading is so simple
    i2c_send_t send = {.slave = sensor->loc.addr, .stop = 1, .len = 1};
    uint8_t send_cmd[sizeof(send) + 1];
    memcpy(send_cmd, &send, sizeof(send));
    send_cmd[sizeof(send)] = PRECISIONS[sensor->precision];
    errno_t result = devctl(sensor->loc.bus, DCMD_I2C_SEND, &send_cmd, sizeof(send_cmd), NULL);
    return_err(result);
    // Wait for the measurement to take place, depends on precision
    usleep(MEASUREMENT_TIMES[sensor->precision]);
    i2c_recv_t read = {.slave = sensor->loc.addr, .stop = 1, .len = 6};
    uint8_t read_cmd[sizeof(send) + 6];
    memcpy(read_cmd, &read, sizeof(read));
    result = devctl(sensor->loc.bus, DCMD_I2C_RECV, &read_cmd, sizeof(read_cmd), NULL);
    return_err(result);

    // For now, just discard one half of the measurement due to timestamping and the sensor interface
    // Should check the CRC in the future
    switch (tag) {
    case TAG_TEMPERATURE: {
        int t_ticks = (read_cmd[sizeof(read) + SHT41_T_MSB] * 256 + read_cmd[sizeof(read) + SHT41_T_LSB]);
        float temp = -45 + 175 * ((float)t_ticks / 65535); // Degrees C
        memcpy(buf, &temp, sizeof(temp));
        *nbytes = sizeof(temp);
        break;
    }
    case TAG_HUMIDITY: {
        int rh_ticks = (read_cmd[sizeof(read) + SHT41_RH_MSB] * 256 + read_cmd[sizeof(read) + SHT41_RH_LSB]);
        float hum = -6 + 125 * ((float)rh_ticks / 65535); // Degrees C
        // Limit values of humidity, since values outside of 0-100
        if (hum < 0) {
            hum = 0;
        } else if (hum > 100) {
            hum = 100;
        }
        // Round, because sensor has accuracy of, at best, +/-1.0% 
        hum = rintf(hum);
        memcpy(buf, &hum, sizeof(hum));
        *nbytes = sizeof(hum);
        break;
    }
    default:
        return EINVAL;
    }
    return EOK;
}

/**
 * Resets the SHT41 sensor
 * @param loc The sensor's location on the I2C bus.
 * @return Any error from the attempted reset. EOK if successful.
 */
static errno_t sht41_reset(SensorLocation *loc) {
    // Reset the sensor
    i2c_send_t reset = {.stop = 1, .len = 1, .slave = loc->addr};
    uint8_t reset_cmd[sizeof(i2c_send_t) + 1];
    memcpy(reset_cmd, &reset, sizeof(reset));
    reset_cmd[sizeof(reset)] = CMD_RESET;
    return devctl(loc->bus, DCMD_I2C_SEND, &reset_cmd, sizeof(reset_cmd), NULL);
}

/**
 * Prepares the SHT41 for reading and initializes the sensor context with the calibration coefficients.
 * @param sensor A reference to an SHT41 sensor.
 * @return The error status of the call. EOK if successful.
 */
static errno_t sht41_open(Sensor *sensor) {
    return sht41_reset(&sensor->loc);
    // Could read the sensor or check serial number if wanted in the future
}

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
    sensor->tag_list = (SensorTagList){.tags = TAGS, .len = sizeof(TAGS) / sizeof(SensorTag)};
    sensor->context.size = 0;
    sensor->open = &sht41_open;
    sensor->read = &sht41_read;
}
