/**
 * @file MAXM10S.c
 * @brief Sensor API interface implementation for the MAXM10S gps sensor.
 *
 * Sensor API interface implementation for the MAXM10S gps sensor which uses I2C communication.
 */

#include "m10spg.h"
#include "../sensor_api.h"
#include <errno.h>
#include <hw/i2c.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** The first preamble synchronization header. */
#define H1 0xB5

/** The second preamble synchronization header. */
#define H2 0x62

static const SensorTag TAGS[] = {TAG_TIME};

/** Commands to configure the I2C interface */
typedef enum i2c_config {
    CFG_I2C_ADDRESS = 0x20510001,         /**< I2C address of the reciever (7 bits) */
    CFG_I2C_EXTENDEDTIMEOUT = 0X10510002, /**< Flag to disable timeouting the interface ofter 1.5s */
    CFG_I2C_ENABLED = 0X10510003          /**< Flag to indicate if the I2C inteface should be enabled*/
} I2cConfig;

/** Inout protocol to enable the I2C interface */
typedef enum i2c_protocol {
    CFG_I2CINPROT_UBX = 0x10710001, /**< Flag to indicate if UBX should be an input protocol on I2C */
    CFG_I2CINPROT_NMEA = 0x10710002 /**< Flag to indicate if NMEA should be an input protocol on I2C */
} I2cProtocol;

/** UBX commands to be used on the GPS sensor */
typedef enum ubx_nav_protocol {
    UBX_NAV_POSLLH = 0x02,   /**< Fetches a 28 byte long block containing geodetic positioning information. */
    UBX_NAV_DOP = 0x04,      /**< Fetches an 18 byte long block containing dilution of precision values */
    UBX_NAV_ODO = 0x09,      /**< Fetches a 20 byte long block containing ground distance information */
    UBX_NAV_RESETODO = 0x10, /**< Resets the odometer. */
    UBX_NAV_TIMEUTC = 0x21,  /**< Fetches a 20 byte long block containing UTC time information. */
    UBX_NAV_SAT = 0x35 /**<Fetches a 8 + #Satellites in view * 12 byte long block containing satellite information. */
} UBXNavProtocolCmds;

typedef struct {
    uint8_t header_1;
    uint8_t header_2;
    uint8_t class;
    uint8_t id;
    uint16_t length;
} UBXHeader;

typedef struct {
    uint8_t version;
    uint8_t reserved[3];
    uint8_t unique_id[6];
} UBXSecUniqidPayload;

typedef struct {
    uint8_t header_1;
    uint8_t header_2;
    uint8_t class;
    uint8_t id;
    uint16_t length;
    UBXSecUniqidPayload payload;
    uint8_t checksum_a;
    uint8_t checksum_b;
} UBXSecUniqId;

/**
 * Writes data to the M10SPG.
 * @param sensor A reference to an M10SPG sensor.
 * @param buf A pointer to the memory location containing the data.
 * @param nbytes The number of bytes to be written to the M10SPG.
 * @return Error status of reading from the sensor. EOK if successful.
 */
static errno_t m10spg_write(Sensor *sensor, void *buf, size_t nbytes) {

    i2c_send_t header = {.stop = 1, .slave = sensor->loc.addr, .len = nbytes};
    uint8_t data[sizeof(header) + nbytes];
    memcpy(data, &header, sizeof(header));
    memcpy(&data[sizeof(header)], buf, nbytes);

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SEND, data, sizeof(header) + nbytes, NULL);

    return err;
}

/**
 * Reads the specified data from the M10SPG.
 * @param sensor A reference to an M10SPG sensor.
 * @param tag The tag of the data type that should be read.
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes that were written into the byte array buffer.
 * @return errno_t The error status of the call. EOK if successful.
 */
static errno_t m10spg_read(Sensor *sensor, const SensorTag tag, void *buf, size_t *nbytes) {

    // Make sure in read mode
    i2c_addr_t read_addr = sensor->loc.addr;

    for (int j = 0; j < 500; j++) {
        i2c_sendrecv_t header = {.stop = 1, .slave = sensor->loc.addr, .recv_len = 3, .send_len = 0};

        uint8_t read_cmd[sizeof(header) + 3];
        memcpy(read_cmd, &header, sizeof(header));

        errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SENDRECV, read_cmd, sizeof(read_cmd), NULL);
        if (err != EOK) return err;

        for (uint8_t i = 0; i < 3; i++) {
            // printf("%x", read_cmd[sizeof(header) + i]);
            putchar(read_cmd[sizeof(header) + i]);
        }
    }
    printf("\n");

    return EOK;
}

/**
 * Returns the number of bytes ready to be read from the sensor
 *
 * @param sensor The sensor to check the number of bytes waiting from
 * @param result The total number of bytes ready to be read
 * @return errno_t The error status of the call. EOK if successful.
 */
static errno_t m10spg_available_bytes(Sensor *sensor, uint16_t *result) {

    // Send the address of the first register, then the second byte read will be the next register (0xFE)
    i2c_send_t header = {.stop = 0, .slave = sensor->loc.addr, .len = 1};
    uint8_t address_cmd[sizeof(header) + 1];
    memcpy(address_cmd, &header, sizeof(header));
    address_cmd[sizeof(header)] = 0xFD;

    i2c_recv_t read_header = {.stop = 1, .slave = sensor->loc.addr, .len = 2};
    uint8_t read_cmd[sizeof(read_header) + 2];
    memcpy(read_cmd, &read_header, sizeof(read_header));

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SEND, address_cmd, sizeof(address_cmd), NULL);
    return_err(err);

    err = devctl(sensor->loc.bus, DCMD_I2C_RECV, read_cmd, sizeof(read_cmd), NULL);
    return_err(err);

    *result = ((uint16_t)read_cmd[sizeof(header)]) * 256 + (uint16_t)(read_cmd[sizeof(header) + 1]);
    return EOK;
}

/**
 * Prepares the M10SPG for reading.
 * @param sensor A reference to an M10SPG sensor.
 * @return errno_t The error status of the call. EOK if successful.
 */
static errno_t m10spg_open(Sensor *sensor) { return EOK; }

/**
 * Initializes a sensor struct with the interface to interact with the M10SPG.
 * @param sensor The sensor interface to be initialized.
 * @param bus The file descriptor of the I2C bus.
 * @param addr The address of the sensor on the I2C bus.
 * @param precision The precision to read measurements with.
 */
void m10spg_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision) {
    sensor->precision = precision;
    sensor->loc = (SensorLocation){.bus = bus, .addr = {.addr = (addr & 0x42), .fmt = I2C_ADDRFMT_7BIT}};
    sensor->tag_list = (SensorTagList){.tags = TAGS, .len = sizeof(TAGS) / sizeof(SensorTag)};
    sensor->context.size = 0;
    sensor->read = &m10spg_read;
    sensor->open = &m10spg_open;

    // Taken from someone else's code example
    uint8_t ubx_mon_ver[] = {H1, H2, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
    uint8_t ubx_mon_hw3[] = {H1, H2, 0x0A, 0x37, 0x00, 0x00, 0x41, 0xcd};

    m10spg_write(sensor, ubx_mon_ver, sizeof(ubx_mon_ver));
    m10spg_read(sensor, TAG_TIME, NULL, 0);
}
