/**
 * @file sht41.h
 * @brief Sensor API implementation for the SHT41 temperature and humidity sensor.
 */
#include "sht41.h"
#include "../../crc-utils/crc.h"
#include "../sensor_api.h"
#include "hw/i2c.h"
#include <math.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/** Defines positions of most and least significant bytes in the sensor's response for temperatue */
#define SHT41_T_MSB 0
#define SHT41_T_LSB 1

/** Defines positions of the most and least significant bytes in the sensor's response for humidity */
#define SHT41_RH_MSB 3
#define SHT41_RH_LSB 4

/** Defines the first byte to include in the CRC calculation */
#define SHT41_T_CRC 0
#define SHT41_RH_CRC 3

/** Defines the number of bytes to be included in the CRC calculation*/
#define SHT41_CRC_LEN 3

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

/** Initial value of the calcualted CRC */
#define SHT41_CRC_INIT 0xFF
#define SHT41_CRC_POLY 0x31

#ifdef SHT41_USE_CRC_LOOKUP
static const CRC8LookupTable CRC_LOOKUP = {
    .table = {
        0x0,  0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e, 0x43, 0x72,
        0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb, 0x98, 0xa9, 0x3e, 0xf,  0x5c, 0x6d, 0x86, 0xb7, 0xe4, 0xd5,
        0x42, 0x73, 0x20, 0x11, 0x3f, 0xe,  0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8, 0xc5, 0xf4, 0xa7, 0x96, 0x1,  0x30,
        0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb, 0x3d, 0xc,  0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa,
        0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71, 0x22, 0x13, 0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6,
        0xa5, 0x94, 0x3,  0x32, 0x61, 0x50, 0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c, 0x2,  0x33, 0x60, 0x51,
        0xc6, 0xf7, 0xa4, 0x95, 0xf8, 0xc9, 0x9a, 0xab, 0x3c, 0xd,  0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4,
        0xe7, 0xd6, 0x7a, 0x4b, 0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x7,  0x36, 0x65, 0x54,
        0x39, 0x8,  0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3, 0x44, 0x75, 0x26, 0x17, 0xfc, 0xcd,
        0x9e, 0xaf, 0x38, 0x9,  0x5a, 0x6b, 0x45, 0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2, 0xbf, 0x8e, 0xdd, 0xec,
        0x7b, 0x4a, 0x19, 0x28, 0x6,  0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91, 0x47, 0x76, 0x25, 0x14, 0x83, 0xb2,
        0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0xb,  0x58, 0x69, 0x4,  0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93,
        0xbd, 0x8c, 0xdf, 0xee, 0x79, 0x48, 0x1b, 0x2a, 0xc1, 0xf0, 0xa3, 0x92, 0x5,  0x34, 0x67, 0x56, 0x78, 0x49,
        0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef, 0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24, 0x15, 0x3b, 0xa,  0x59, 0x68,
        0xff, 0xce, 0x9d, 0xac,
    }};
#endif

/**
 * Checks if the calculated CRC for this section of data is equal to zero
 * @param start The start of the data which ends with it's own 8-bit CRC
 * @param nbytes The length of the data to be checked, including the CRC
 * @return errno_t EBADMSG
 */
static inline errno_t check_crc(uint8_t *buf, size_t nbytes) {
#ifdef SHT41_USE_CRC_LOOKUP
    // Check the CRC to make sure we're not about to read garbage
    if (calculate_crc8(buf, nbytes, &CRC_LOOKUP, SHT41_CRC_INIT) != 0) {
        return EBADMSG;
    }
#else
    if (calculate_crc8_bitwise(buf, nbytes, SHT41_CRC_POLY, SHT41_CRC_INIT) != 0) {
        return EBADMSG;
    }
#endif
    else {
        return EOK;
    }
}

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

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SEND, &send_cmd, sizeof(send_cmd), NULL);
    if (err != EOK) {
        return err;
    };

    // Wait for the measurement to take place, depends on precision
    usleep(MEASUREMENT_TIMES[sensor->precision]);
    i2c_recv_t read = {.slave = sensor->loc.addr, .stop = 1, .len = 6};
    uint8_t read_cmd[sizeof(send) + 6];
    memcpy(read_cmd, &read, sizeof(read));
    err = devctl(sensor->loc.bus, DCMD_I2C_RECV, &read_cmd, sizeof(read_cmd), NULL);
    if (err != EOK) {
        return err;
    };

    // For now, just discard one half of the measurement due to timestamping and the sensor interface
    switch (tag) {
    case TAG_TEMPERATURE: {
        err = check_crc(read_cmd + sizeof(read) + SHT41_T_CRC, SHT41_CRC_LEN);
        if (err != EOK) return err;

        int t_ticks = (read_cmd[sizeof(read) + SHT41_T_MSB] * 256 + read_cmd[sizeof(read) + SHT41_T_LSB]);
        float temp = -45 + 175 * ((float)t_ticks / 65535); // Degrees C
        memcpy(buf, &temp, sizeof(temp));
        *nbytes = sizeof(temp);
        break;
    }
    case TAG_HUMIDITY: {
        err = check_crc(read_cmd + sizeof(read) + SHT41_RH_CRC, SHT41_CRC_LEN);
        if (err != EOK) return err;

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
    return err;
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
