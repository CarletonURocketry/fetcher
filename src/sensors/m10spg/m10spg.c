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
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
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

/** UBX header for all UBX protocol messages sent to the reciever */
typedef struct {
    uint8_t sync_1;
    uint8_t sync_2;
    uint8_t class;
    uint8_t id;
    uint16_t length;
} UBXHeader;

/** UBX protcol style message, can be sent directly to the reciever */
typedef struct {
    UBXHeader header;   /** A UBX protocol header*/
    void *payload;      /** The payload of the message (length is stored in the header) */
    uint8_t checksum_a; /** The first checksum byte of the message, including all fields past the synch characters */
    uint8_t checksum_b; /** The second checksum byte */
} UBXProtocolMsg;

/** A struct representing the payload of a UBX-NAV-TIMEUTC message */
typedef struct {
    uint32_t iTOW;
    uint32_t tAcc;
    int32_t nano;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t min;
    uint8_t sec;
    uint8_t flags;
} UBXUTCPayload;

typedef struct {
    uint8_t version;
    uint8_t reserved[3];
    uint8_t unique_id[6];
} UBXSecUniqidPayload;

/**
 * Gets the total length of a message, including checksums and the header
 * @return uint16_t
 */
static inline uint16_t ubx_message_length(UBXProtocolMsg *msg) { return sizeof(msg->header) + msg->header.length + 2; }

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
    memcpy(data + sizeof(header), buf, nbytes);

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SEND, data, sizeof(header) + nbytes, NULL);

    return err;
}

/**
 * Reads the specified data from the M10SPG.
 * @param sensor A reference to an M10SPG sensor.
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes to read into the buffer
 * @return errno_t The error status of the call. EOK if successful.
 */
static errno_t m10spg_read(Sensor sensor, const SensorTag tag, void *buf, size_t *nbytes) {}

/**
 * Reads a certain number of bytes into the specified buffer
 * @param sensor A reference to an M10SPG sensor.
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes to read into the buffer
 * @return errno_t The error status of the call. EOK if successful.
 */
static errno_t m10spg_read_bytes(Sensor *sensor, void *buf, size_t nbytes) {
    i2c_recv_t header = {.stop = 1, .slave = sensor->loc.addr, .len = nbytes};
    uint8_t read_cmd[sizeof(header) + nbytes];
    memcpy(read_cmd, &header, sizeof(header));

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_RECV, read_cmd, sizeof(read_cmd), NULL);
    if (err != EOK) return err;

    // Copy out what we recieved
    memcpy(buf, read_cmd + sizeof(header), nbytes);
    return EOK;
}

/**
 * Gets the next ublox protcol message from the reciever, reading through any non-ublox data
 * @param sensor The sensor to get the message from
 * @param msg An empty message structure, with a payload pointing at a data buffer to read the payload into
 * @param max_payload The maximum size that the payload can be (the size of the buffer pointed to by it)
 * @param timeout THe maximum time to try and get a message
 * @return errno_t EINVAL if the buffer is too small for the message found. ETIMEOUT if the timeout expires before a
 * message is found. EBADMSG if the second sync char is not valid. EOK otherwise.
 */
static errno_t recv_ubx_message(Sensor *sensor, UBXProtocolMsg *msg, uint16_t max_payload, uint8_t timeout) {
    struct timespec start, stop;
    clock_gettime(CLOCK_REALTIME, &start);
    clock_gettime(CLOCK_REALTIME, &stop);
    while ((stop.tv_sec - start.tv_sec) < timeout) {
        errno_t err = m10spg_read_bytes(sensor, &msg->header.sync_1, sizeof(msg->header.sync_1));
        if (err != EOK) return err;

        // Make sure we're at the start of a new message
        if (msg->header.sync_1 == H1) {
            err = m10spg_read_bytes(sensor, &msg->header.sync_2, sizeof(msg->header.sync_2));
            if (err != EOK) return err;
            if (msg->header.sync_2 == H2) {
                // Found something. Get message class, id, and length
                err =
                    m10spg_read_bytes(sensor, &msg->header.class,
                                      sizeof(msg->header.class) + sizeof(msg->header.id) + sizeof(msg->header.length));
                if (err != EOK) return err;

                printf("Found a message length : %d\n", msg->header.length);
                // Make sure the space we allocated for the payload is big enough
                if (msg->header.length > max_payload) {
                    return EINVAL;
                }
                // Read payload
                err = m10spg_read_bytes(sensor, msg->payload, msg->header.length);
                if (err != EOK) return err;

                // Read in the checksums (assume contiguous)
                err = m10spg_read_bytes(sensor, &msg->checksum_a, 2);
                return err;
            } else {
                return EBADMSG;
            }
        }
        clock_gettime(CLOCK_REALTIME, &stop);
        usleep(10000);
    }
    return ETIMEDOUT;
}

/**
 * Prints a populated message structure to standard output
 * @param msg The message to print. Prints the fields as hexidecimal
 */
static void print_ubx_message(UBXProtocolMsg *msg) {
    printf("Class: %x, ID: %x, Length: %x, Payload: ", msg->header.class, msg->header.id, msg->header.length);
    for (uint8_t *byte = msg->payload; byte < (uint8_t *)msg->payload + msg->header.length; byte++) {
        printf("%x ", *byte);
    }
    putchar('\n');
    for (uint8_t *byte = msg->payload; byte < (uint8_t *)msg->payload + msg->header.length; byte++) {
        putchar(*byte);
    }
    putchar('\n');
}

static errno_t send_ubx_message(Sensor *sensor, UBXProtocolMsg *msg) {
    i2c_send_t header = {.stop = 1, .slave = sensor->loc.addr, .len = ubx_message_length(msg)};
    uint8_t data[sizeof(header) + ubx_message_length(msg)];
    memcpy(data, &header, sizeof(header));
    memcpy(data + sizeof(header), &msg->header, sizeof(msg->header));
    memcpy(data + sizeof(header) + sizeof(msg->header), msg->payload, msg->header.length);
    memcpy(data + sizeof(header) + sizeof(msg->header) + msg->header.length, &msg->checksum_a, 2);

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SEND, data, sizeof(data), NULL);
    return err;
}

static errno_t dump_buffer(Sensor *sensor, size_t bytes) {
    uint8_t buff[bytes];
    m10spg_read_bytes(sensor, buff, bytes);
    for (size_t i = 0; i < bytes; i++) {
        printf("%x ", buff[i]);
    }
    putchar('\n');
    for (size_t i = 0; i < bytes; i++) {
        putchar(buff[i]);
    }
    putchar('\n');
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
 * The loop portion of a Fletcher-8 checksum (use set_ubx_checksum for calculating checksums)
 */
static inline void calculate_partial_checksum(uint8_t *start, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    for (uint8_t *byte = start; byte < start + len; byte++) {
        *ck_a = *ck_a + *byte;
        *ck_b = *ck_b + *ck_a;
    }
}

/**
 * Set the checksum on a message that already has had it's header initialized
 * @param msg The message to calculate and set a checksum on. Length in header must be initialized to the size of the
 * payload
 * @return errno_t EINVAL if the header is invalid and the checksum was not initialized
 */
static errno_t set_ubx_checksum(UBXProtocolMsg *msg) {
    msg->checksum_a = 0;
    msg->checksum_b = 0;

    calculate_partial_checksum((uint8_t *)(&msg->header.class), 4, &msg->checksum_a, &msg->checksum_b);
    // Payload may not be contiguous, so calculate the checksum in two halves
    calculate_partial_checksum((uint8_t *)msg->payload, msg->header.length, &msg->checksum_a, &msg->checksum_b);
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
    sensor->open = &m10spg_open;
    /*
    uint8_t ubx_mon_ver[] = {H1, H2, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
    uint8_t ubx_mon_hw3[] = {H1, H2, 0x0A, 0x37, 0x00, 0x00, 0x41, 0xcd};
    uint8_t ubx_mon_gnss[] = {H1, H2, 0x0a, 0x28, 0x00, 0x00, 0x32, 0xa0};
    uint8_t ubx_mon_comms[] = {H1, H2, 0x0a, 0x36, 0x00, 0x00, 0x40, 0xca};
    uint32_t key = 0x10510003;
    uint8_t *key_bytes = (uint8_t *)&key;
    uint8_t ubx_i2c_enabled[] = {H1,   H2,   0x06, 0x8B, 0x08, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x03, 0x00, 0x51, 0x10, 0xfd, 0x4f};
    */
    uint8_t payload_buffer[300];
    UBXProtocolMsg mon_ver_poll;
    mon_ver_poll.header.sync_1 = H1;
    mon_ver_poll.header.sync_2 = H2;
    mon_ver_poll.header.class = 0x0A;
    mon_ver_poll.header.id = 0x04;
    mon_ver_poll.header.length = 0;

    mon_ver_poll.payload = payload_buffer;
    set_ubx_checksum(&mon_ver_poll);

    errno_t err = send_ubx_message(sensor, &mon_ver_poll);
    if (err != EOK) printf("Error here: %d, Code %d\n", __LINE__, err);
    sleep(1);

    // Use this to see the ascii and hex of just the ublox message (it looks for the sync chars)
    err = recv_ubx_message(sensor, &mon_ver_poll, 300, 2);
    if (err != EOK) printf("Error here: %d, Code %d\n", __LINE__, err);
    print_ubx_message(&mon_ver_poll);

    // See buffer contents in hex and ascii - one big ol read
    // dump_buffer(sensor, 600);
}
