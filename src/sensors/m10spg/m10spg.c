/**
 * @file MAXM10S.c
 * @brief Sensor API interface implementation for the MAXM10S gps sensor.
 *
 * Sensor API interface implementation for the MAXM10S gps sensor which uses I2C communication.
 */

#include "../sensor_api.h"
#include <assert.h>
#include <errno.h>
#include <hw/i2c.h>
#include <string.h>
#include <unistd.h>

#define DCMD_I2C_SENDRECV __DIOTF(_DCMD_I2C, 7, i2c_sendrecv_t)

// Sets the address of the gps on the board
uint8_t gps_address = 0x42;

// static const SensorTag TAGS[] = {TAG_DOP, TAG_ODOMETER, TAG_POSITION, TAG_RESET, TAG_SATELLITE, TAG_TIME};

// Commands to configure the I2C interface
typedef enum i2c_config {
    CFG_I2C_ADDRESS = 0x20510001,         /* I2C address of the reciever (7 bits) */
    CFG_I2C_EXTENDEDTIMEOUT = 0X10510002, /* Flag to disable timeouting the interface ofter 1.5s */
    CFG_I2C_ENABLED = 0X10510003          /* Flag to indicate if the I2C inteface should be enabled*/
} I2cConfig;

// Inout protocol to enable the I2C interface
typedef enum i2c_protocol {
    CFG_I2CINPROT_UBX = 0x10710001, /* Flag to indicate if UBX should be an input protocol on I2C */
    CFG_I2CINPROT_NMEA = 0x10710002 /* Flag to indicate if NMEA should be an input protocol on I2C */
} I2cProtocol;

// UBX commands to be used on the GPS sensor
typedef enum ubx_nav_protocol {
    UBX_NAV_POSLLH = 0x02,   /*Fetches a 28 byte long block containing geodetic positioning information.*/
    UBX_NAV_DOP = 0x04,      /*Fetches an 18 byte long block containing dilution of precision values*/
    UBX_NAV_ODO = 0x09,      /*Fetches a 20 byte long block containing ground distance information*/
    UBX_NAV_RESETODO = 0x10, /*Resets the odometer.*/
    UBX_NAV_TIMEUTC = 0x21,  /*Fetches a 20 byte long block containing UTC time information.*/
    UBX_NAV_SAT = 0x35 /*Fetches a 8 + #Satellites in view * 12 byte long block containing satellite information.*/
} UBXNavProtocolCmds;

// Test struct to see if the message receiving format works.
typedef struct {

    /*UBX header 1 as a byte array*/
    uint8_t header_1;
    /*UBX header 2 as a byte array*/
    uint8_t header_2;
    /*UBX class as a byte array*/
    uint8_t class;
    /*UBX id a as a byte array*/
    uint8_t id;
    /*UBX length a as a byte array*/
    uint16_t length;
    /*UBX checksum a as a byte array*/
    uint8_t checksum_a;
    /*UBX checksum b as a byte array*/
    uint8_t checksum_b;

} UBXHeader;

typedef struct {

    uint8_t version;
    uint8_t reserved[3];
    uint8_t unique_id[6];

} UBXSecUniqidPayload;

typedef struct {

    /*UBX header 1 as a byte array*/
    uint8_t header_1;
    /*UBX header 2 as a byte array*/
    uint8_t header_2;
    /*UBX class as a byte array*/
    uint8_t class;
    /*UBX id a as a byte array*/
    uint8_t id;
    /*UBX length a as a byte array*/
    uint16_t length;
    UBXSecUniqidPayload payload;
    /*UBX checksum a as a byte array*/
    uint8_t checksum_a;
    /*UBX checksum b as a byte array*/
    uint8_t checksum_b;

} UBXSecUniqId;

static UBXSecUniqidPayload m10spg_recv(Sensor *sensor) {

    i2c_sendrecv_t read_id_cmd_hdr = {.slave = sensor->loc.addr, .send_len = 6, .recv_len = 16};
    uint8_t read_id_cmd[sizeof(read_id_cmd_hdr) + 16];
    memcpy(read_id_cmd, &read_id_cmd_hdr, sizeof(read_id_cmd_hdr));
    memcpy(read_id_cmd[sizeof(read_id_cmd_hdr)], &read_id_cmd, sizeof(6));

    errno_t err = devctl(DCMD_I2C_SENDRECV, read_id_cmd, sizeof(read_id_cmd), NULL, NULL);
    assert(err == EOK);

    UBXSecUniqId *ret = (UBXSecUniqId *)(&read_id_cmd[sizeof(read_id_cmd_hdr)]);
    ret->payload.unique_id[0];
    ret->payload.version;

    return ret->payload;
}

// static errno_t m10spg_init(Sensor *sensor, const int bus, const uint8_t addr) {

//    sensor->loc = (SensorLocation){.bus = bus, .addr = {.addr = (addr & 0x42), .fmt = I2C_ADDRFMT_7BIT}};

//}
