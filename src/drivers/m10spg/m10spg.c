/**
 * @file MAXM10S.c
 * @brief Sensor API interface implementation for the MAXM10S gps sensor.
 *
 * Sensor API interface implementation for the MAXM10S gps sensor which uses I2C communication.
 */

#include "m10spg.h"
#include "../sensor_api.h"
#include "ubx_def.h"
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

/** How long the recieve command for UBX messages should wait between trying to read a message, in usec */
#define RECV_SLEEP_TIME 10000

/** How long m10spg_read should wait for a response in seconds */
#define DEFAULT_TIMEOUT 1

static const SensorTag TAGS[] = {TAG_TIME};

/** Pre-built frame for polling the UBX-MON-VER message */
static const UBXFrame POLL_MON_VER = {
    .header = {.class = 0x0A, .id = 0x04, .length = 0x00}, .checksum_a = 0x0E, .checksum_b = 0x34};

/** Pre-built frame for polling the UBX-NAV-UTC message */
static const UBXFrame POLL_NAV_UTC = {
    .header = {.class = 0x01, .id = 0x21, .length = 0x00}, .checksum_a = 0x22, .checksum_b = 0x67};

/** Pre-built frame for polling the UBX-NAV-STAT message */
static const UBXFrame POLL_NAV_STAT = {
    .header = {.class = 0x01, .id = 0x03, .length = 0x00}, .checksum_a = 0x04, .checksum_b = 0x0d};

/** Pre-built frame for polling the UBX-NAV-POSLLH message */
static const UBXFrame POLL_NAV_POSLLH = {
    .header = {.class = 0x01, .id = 0x02, .length = 0x00}, .checksum_a = 0x03, .checksum_b = 0x0a};

/** Pre-built frame for polling the UBX-NAV-VELNED message */
static const UBXFrame POLL_NAV_VELNED = {
    .header = {.class = 0x01, .id = 0x12, .length = 0x00}, .checksum_a = 0x13, .checksum_b = 0x3a};

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
 * A function for debugging the interface with the M10SPG module, prints a number of bytes from the I2C buffer,
 * displaying the bytes in hex and ascii. If the buffer is empty, a hex value of 0XFF is printed
 * @param sensor The sensor whos read buffer will be dumped to stdout
 * @param bytes The number of bytes to read from the buffer (can be greater than the number of bytes actually in the
 * buffer)
 * @return errno_t The status of the read operation, EOK if successful
 */
static errno_t debug_dump_buffer(Sensor *sensor, size_t bytes) {
    uint8_t buff[bytes];
    errno_t err = m10spg_read_bytes(sensor, buff, bytes);
    return_err(err);
    for (size_t i = 0; i < bytes; i++) {
        printf("%x ", buff[i]);
    }
    putchar('\n');
    for (size_t i = 0; i < bytes; i++) {
        putchar(buff[i]);
    }
    putchar('\n');
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
    clock_gettime(CLOCK_MONOTONIC, &start);
    do {
        uint8_t sync = 0;
        errno_t err = m10spg_read_bytes(sensor, &sync, sizeof(sync));
        return_err(err);

        // Make sure we're at the start of a new message
        if (sync == SYNC_ONE) {
            err = m10spg_read_bytes(sensor, &sync, sizeof(sync));
            return_err(err);
            if (sync == SYNC_TWO) {
                // Found something. Get message class, id, and length
                err =
                    m10spg_read_bytes(sensor, &msg->header.class,
                                      sizeof(msg->header.class) + sizeof(msg->header.id) + sizeof(msg->header.length));
                return_err(err);
                printf("Found a message length : %d\n", msg->header.length);
                // Make sure the space we allocated for the payload is big enough
                if (msg->header.length > max_payload) {
                    return EINVAL;
                }
                // Read payload
                err = m10spg_read_bytes(sensor, msg->payload, msg->header.length);
                return_err(err);
                // Read in the checksums (assume contiguous)
                err = m10spg_read_bytes(sensor, &msg->checksum_a, 2);
                return err;
            } else {
                return EBADMSG;
            }
        }
        // Sleeping gives time for the buffer to be opened, if it has closed (timeout occurs after 1.5s)
        usleep(RECV_SLEEP_TIME);
        // Get the time now
        clock_gettime(CLOCK_MONOTONIC, &stop);
    } while ((stop.tv_sec - start.tv_sec) < timeout);
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
 * Prints a populated message structure to standard output, which is useful when creating a pre-built message
 * @param msg The message to print. Prints the fields as hexidecimal and ascii
 */
static void debug_print_ubx_message(const UBXFrame *msg) {
    printf("Class: %x, ID: %x, Length: %x, Payload:", msg->header.class, msg->header.id, msg->header.length);
    for (uint8_t *byte = msg->payload; byte < (uint8_t *)msg->payload + msg->header.length; byte++) {
        printf(" %x", *byte);
    }
    printf(", Checksum: %x %x\n", msg->checksum_a, msg->checksum_b);
    for (uint8_t *byte = msg->payload; byte < (uint8_t *)msg->payload + msg->header.length; byte++) {
        putchar(*byte);
    }
    putchar('\n');
}

/**
 * Tries to read a UBX message from the buffer within a certain time limit, and if one exists, prints its contents
 * @param sensor The sensor to read a UBX message from
 * @param max_bytes The maximum number of bytes the message's payload can have
 * @param timeout The maximum time to wait for a message in the buffer
 * @return errno_t The status of the read operation on the buffer, EOK if successful
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
static errno_t m10spg_read(Sensor *sensor, const SensorTag tag, void *buf, size_t *nbytes) { return EOK; }

/**
 * Prepares the M10SPG for reading.
 * @param sensor A reference to an M10SPG sensor.
 * @return errno_t The error status of the call. EOK if successful.
 */
static errno_t m10spg_open(Sensor *sensor) {
    UBXFrame msg;
    UBXValsetPayload valset_payload;
    UBXAckPayload ack_payload;

    // Configure the chip
    msg.payload = &valset_payload;
    init_valset_message(&msg, RAM_LAYER);
    uint8_t config_disabled = 0;
    // Disable NMEA output on I2C
    add_valset_item(&msg, (uint32_t)0x10720002, &config_disabled, UBX_TYPE_L);
    // Disable NMEA input on I2C
    add_valset_item(&msg, (uint32_t)0x10710002, &config_disabled, UBX_TYPE_L);
    set_ubx_checksum(&msg);

    errno_t err = send_ubx_message(sensor, &msg);
    return_err(err);

    // Check if configuration was successful
    msg.payload = &ack_payload;
    err = recv_ubx_message(sensor, &msg, sizeof(ack_payload), 1);
    return_err(err);
    if (msg.header.class == 0x05) {
        if (msg.header.id == 0x01) {
            return EOK;
        } else if (msg.header.id == 0x00) {
            // Valset was not successful, check interface manual for possible reasons
            return EINVAL;
        }
    }
    // Some other response interrupted our exchange
    return EINTR;
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