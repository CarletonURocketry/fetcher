/**
 * @file MAXM10S.c
 * @brief Sensor API interface implementation for the MAXM10S gps sensor.
 *
 * Sensor API interface implementation for the MAXM10S gps sensor which uses I2C communication.
 */

#include "m10spg.h"
#include "../sensor_api.h"
#include <assert.h>
#include <errno.h>
#include <hw/i2c.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

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
    uint8_t checksum_a;
    uint8_t checksum_b;
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

static UBXSecUniqidPayload m10spg_recv(Sensor *sensor) {

    UBXHeader ubx_header = {
        .header_1 = H1,
        .header_2 = H2,
        .id = 0x03,
        .class = 0x27,
        .length = 0,
        .checksum_a = 0,
        .checksum_b = 0,
    };

    // Calculate checksum
    for (uint8_t i = 0; i < sizeof(ubx_header) - 2; i++) {
        ubx_header.checksum_a += ((uint8_t *)(&ubx_header))[i];
        ubx_header.checksum_b += ubx_header.checksum_a;
    }

    i2c_sendrecv_t read_id_cmd_hdr = {
        .slave = sensor->loc.addr, .send_len = sizeof(ubx_header), .recv_len = sizeof(UBXSecUniqId)};
    uint8_t read_id_cmd[sizeof(read_id_cmd_hdr) + sizeof(UBXSecUniqId)];
    memcpy(read_id_cmd, &read_id_cmd_hdr, sizeof(read_id_cmd_hdr));

    memcpy(&read_id_cmd[sizeof(read_id_cmd_hdr)], &ubx_header, sizeof(ubx_header));

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SENDRECV, read_id_cmd, sizeof(read_id_cmd), NULL);
    assert(err == EOK);

    UBXSecUniqId *ret = (UBXSecUniqId *)(&read_id_cmd[sizeof(read_id_cmd_hdr)]);

    printf("C_A: %u\n", ret->checksum_a);
    printf("C_B: %u\n", ret->checksum_b);
    printf("h1: %u\n", ret->header_1);
    printf("h2: %u\n", ret->header_2);
    printf("id: %u\n", ret->id);
    printf("class: %u\n", ret->class);
    printf("length: %u\n", ret->length);

    for (int i = 0; i < 6; i++) {
        printf("Payload ID: %u\n", ret->payload.unique_id[i]);
    }

    return ret->payload;
}

void m10spg_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision) {
    sensor->precision = precision;
    sensor->loc = (SensorLocation){.bus = bus, .addr = {.addr = (addr & 0x42), .fmt = I2C_ADDRFMT_7BIT}};
    sensor->tag_list = (SensorTagList){.tags = TAGS, .len = sizeof(TAGS) / sizeof(SensorTag)};
    sensor->context.size = 0;
    m10spg_recv(sensor);
}
