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
#define SYNC_ONE 0xB5

/** The second preamble synchronization header. */
#define SYNC_TWO 0x62

/** How long the recieve command for UBX messages should wait between trying to read a message*/
#define RECV_SLEEP_TIME 10000

/** How long m10spg_read should wait for a response in seconds */
#define DEFAULT_TIMEOUT 10

static const SensorTag TAGS[] = {TAG_TIME};

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
} UBXFrame;

/** Pre-built frame for polling the UBX-MON-VER message */
static const UBXFrame POLL_MON_VER = {
    .header = {.class = 0x0A, .id = 0x04, .length = 0}, .checksum_a = 0x0E, .checksum_b = 0x34};

/** Pre-built frame for polling the UBX-NAV-UTC message */
static const UBXFrame POLL_NAV_UTC = {
    .header = {.class = 0x01, .id = 0x21, .length = 0}, .checksum_a = 0x22, .checksum_b = 0x67};

/** A struct representing the payload of a UBX-NAV-TIMEUTC message */
typedef struct {
    uint32_t iTOW;
    uint32_t tAcc;
    int32_t nano;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t flags;
} UBXUTCPayload;

/** A struct representing the configuration layer selected in a configuration message (valset or valget) */
typedef enum {
    RAM_LAYER = 0x01,   /** The current configuration - cleared if the reciever enters power saving mode */
    BBR_LAYER = 0x02,   /** The battery backed memory configuration - not cleared unless the backup battery removed */
    FLASH_LAYER = 0x04, /** The flash configuration - does not exist on the M10 MAX */
} UBXConfigLayer;

/** An enum representing the different sizes of values that a configuration message can contain */
typedef enum {
    UBX_TYPE_L = 1,  /** One bit, occupies one byte */
    UBX_TYPE_U1 = 1, /** One byte */
    UBX_TYPE_U2 = 2, /** Two bytes, little endian */
    UBX_TYPE_U4 = 4, /** Four bytes, little endian (excluding U8 because it's not used) */
} UBXValueType;

/* Defines the maximum number of bytes to be used for a valset payload's configuration items (64 items max)*/
#define MAX_VALSET_ITEM_BYTES 20
/** A struct representing the payload of the UBX-VALSET message */
typedef struct {
    uint8_t version;     /** The version of the message (always 0) */
    uint8_t layer;       /** The layer of this config, one of the UBXConfigLayer (typed to ensure one byte) */
    uint8_t reserved[2]; /** Reserved bytes */
    uint8_t config_items[MAX_VALSET_ITEM_BYTES]; /** An array of keys and value pairs */
} UBXValsetPayload;

typedef struct {
    uint8_t version;
    uint8_t reserved[3];
    uint8_t unique_id[6];
} UBXSecUniqidPayload;

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
 * Gets the total length of a message, including checksums and sync characters
 * @return uint16_t
 */
static inline uint16_t ubx_message_length(const UBXFrame *msg) {
    return sizeof(msg->header) + 2 + msg->header.length + 2;
}

/**
 * Prepares the message to be sent again, resetting the length to 0
 * @param msg
 */
static inline void reset_ubx_message(UBXFrame *msg) { msg->header.length = 0; }

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
static errno_t set_ubx_checksum(UBXFrame *msg) {
    msg->checksum_a = 0;
    msg->checksum_b = 0;

    calculate_partial_checksum((uint8_t *)(&msg->header), 4, &msg->checksum_a, &msg->checksum_b);
    // Payload may not be contiguous, so calculate the checksum in two halves
    calculate_partial_checksum((uint8_t *)msg->payload, msg->header.length, &msg->checksum_a, &msg->checksum_b);
}

/**
 * Initializes a valset message, but does not add any configuration items to set
 * @param msg A message with a UBXValsetPayload as its payload
 * @param layer The layer this valset message will configure its items at
 */
static void init_valset_message(UBXFrame *msg, UBXConfigLayer layer) {
    msg->header.class = 0x06;
    msg->header.id = 0x8a;

    UBXValsetPayload *payload = ((UBXValsetPayload *)msg->payload);
    payload->version = 0x00;
    payload->layer = (uint8_t)layer;

    // Set up the message with no items, configure those later and change the length then
    msg->header.length =
        sizeof(msg->header.class) + sizeof(msg->header.id) + sizeof(payload->layer) + sizeof(payload->version);
}

/**
 * Add a configuration item to a valset payload
 * @param msg A UBXFrame with a UBXValsetPayload as its payload
 * @param key A key (in little endian) of the item to set
 * @param value The configuration value to set the key to
 * @param type The type of value being set
 * @return errno_t EINVAL if the message has no space for this configuration item, EOK otherwise
 */
static errno_t add_valset_item(UBXFrame *msg, uint32_t key, const void *value, UBXValueType type) {
    UBXValsetPayload *payload = ((UBXValsetPayload *)msg->payload);
    // Get the next empty byte in the payload's configuration item array
    uint8_t next_byte = msg->header.length - (sizeof(msg->header.class) + sizeof(msg->header.id) +
                                              sizeof(payload->layer) + sizeof(payload->version));
    // Return error if no space left to place the key
    if ((next_byte + sizeof(key) + type) > sizeof(payload->config_items)) {
        return EINVAL;
    }
    memcpy(payload->config_items + next_byte, &key, sizeof(key));
    memcpy(payload->config_items + next_byte + sizeof(key), value, type);
    msg->header.length += sizeof(key) + type;
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
static errno_t recv_ubx_message(Sensor *sensor, UBXFrame *msg, uint16_t max_payload, uint8_t timeout) {
    struct timespec start, stop;
    clock_gettime(CLOCK_REALTIME, &start);
    clock_gettime(CLOCK_REALTIME, &stop);
    while ((stop.tv_sec - start.tv_sec) < timeout) {
        uint8_t sync = 0;
        errno_t err = m10spg_read_bytes(sensor, &sync, sizeof(sync));
        if (err != EOK) return err;

        // Make sure we're at the start of a new message
        if (sync == SYNC_ONE) {
            err = m10spg_read_bytes(sensor, &sync, sizeof(sync));
            if (err != EOK) return err;
            if (sync == SYNC_TWO) {
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
        // Sleeping gives time for the buffer to be opened, if it has closed (timeout occurs after 1.5s)
        usleep(RECV_SLEEP_TIME);
    }
    return ETIMEDOUT;
}

/**
 * Returns the number of bytes ready to be read from the sensor. Currently does not exhibit the expected behaviour and
 * we don't know why, try this again later
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

static errno_t send_ubx_message(Sensor *sensor, const UBXFrame *msg) {
    i2c_send_t header = {.stop = 1, .slave = sensor->loc.addr, .len = ubx_message_length(msg)};
    uint8_t data[sizeof(header) + ubx_message_length(msg)];
    memcpy(data, &header, sizeof(header));
    // Add sync characters
    data[sizeof(header)] = SYNC_ONE;
    data[sizeof(header) + 1] = SYNC_TWO;
    memcpy(data + 2 + sizeof(header), &msg->header, sizeof(msg->header));
    memcpy(data + 2 + sizeof(header) + sizeof(msg->header), msg->payload, msg->header.length);
    memcpy(data + 2 + sizeof(header) + sizeof(msg->header) + msg->header.length, &msg->checksum_a, 2);

    errno_t err = devctl(sensor->loc.bus, DCMD_I2C_SEND, data, sizeof(data), NULL);
    return err;
}

/**
 * Prints a populated message structure to standard output
 * @param msg The message to print. Prints the fields as hexidecimal
 */
static void debug_print_ubx_message(const UBXFrame *msg) {
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

static errno_t debug_dump_buffer(Sensor *sensor, size_t bytes) {
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
 *
 * @param sensor
 * @param max_bytes
 * @param timeout
 * @return errno_t
 */
static errno_t debug_print_next_ubx(Sensor *sensor, size_t max_bytes, size_t timeout) {
    uint8_t buffer[max_bytes];
    UBXFrame msg;
    msg.payload = buffer;
    errno_t err = recv_ubx_message(sensor, &msg, max_bytes, timeout);
    return_err(err);
    debug_print_ubx_message(&msg);
    return EOK;
}

/**
 * Writes data to the M10SPG. Currently useless - UBX messages should be written using the send_ubx_message function
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

    return devctl(sensor->loc.bus, DCMD_I2C_SEND, data, sizeof(header) + nbytes, NULL);
}

/**
 * Reads the specified data from the M10SPG.
 * @param sensor A reference to an M10SPG sensor.
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes to read into the buffer
 * @return errno_t The error status of the call. EOK if successful.
 */
static errno_t m10spg_read(Sensor *sensor, const SensorTag tag, void *buf, size_t *nbytes) {
    errno_t err = EOK;
    // switch (tag) { case TAG_TIME: }
    send_ubx_message(sensor, &POLL_NAV_UTC);
    UBXUTCPayload payload;
    UBXFrame recieved;
    recieved.payload = &payload;
    err = recv_ubx_message(sensor, &recieved, sizeof(payload), DEFAULT_TIMEOUT);
    return_err(err);

    // May be invalid
    if (payload.flags & 0x04) {
        printf("Invalid payload\n");
        return EINVAL;
    }
    printf("GPS UTC: %d, %d, %d, %d, %d\n", payload.year, payload.day, payload.hour, payload.min, payload.sec);
    struct tm recv_time = {.tm_year = 1900 - payload.year,
                           .tm_mon = payload.month - 1,
                           .tm_mday = payload.day,
                           .tm_hour = payload.hour,
                           .tm_min = payload.min,
                           .tm_sec = payload.sec};
    time_t utc_time = mktime(&recv_time);

    return EOK;
}

/**
 * Prepares the M10SPG for reading.
 * @param sensor A reference to an M10SPG sensor.
 * @return errno_t The error status of the call. EOK if successful.
 */
static errno_t m10spg_open(Sensor *sensor) {
    UBXFrame config_msg;
    UBXValsetPayload payload;
    config_msg.payload = &payload;

    init_valset_message(&config_msg, RAM_LAYER);
    uint8_t config_disabled = 0;
    add_valset_item(&config_msg, (uint32_t)0x10720002, &config_disabled, UBX_TYPE_L);
    debug_print_ubx_message(&config_msg);
    set_ubx_checksum(&config_msg);
    errno_t err = send_ubx_message(sensor, &config_msg);
    return_err(err);
    return debug_print_next_ubx(sensor, 300, 10);
}

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
    sensor->read = &m10spg_read;
}