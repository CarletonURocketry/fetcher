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
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifdef __M10SPG_DEBUG__
#include <stdio.h>
#endif // __M10SPG_DEBUG__

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** The first preamble synchronization header. */
#define SYNC_ONE 0xB5

/** The second preamble synchronization header. */
#define SYNC_TWO 0x62

/** How long the recieve command for UBX messages should wait between trying to read a message, in usec */
#define RECV_SLEEP_TIME 1000

/** How long m10spg_read should wait for a response in seconds */
#define DEFAULT_TIMEOUT 2

/** The confirmation value for the platform model that corresponds to an airborne vehicle doing <4G of acceleration */
#define DYNMODEL_AIR_4G 8

/** The nominal time between gps measurements in milliseconds */
#define NOMINAL_MEASUREMENT_RATE 300

/** A configuration key for enabling or disabling periodic message output of the UBX-NAV-PVT message */
#define MSGOUT_I2C_NAV_PVT 0x20910006

static const UBXFrame PREMADE_MESSAGES[] = {
    [UBX_NAV_UTC] = {.header = {.class = 0x01, .id = 0x21, .length = 0x00}, .checksum_a = 0x22, .checksum_b = 0x67},
    [UBX_NAV_POSLLH] = {.header = {.class = 0x01, .id = 0x02, .length = 0x00}, .checksum_a = 0x03, .checksum_b = 0x0a},
    [UBX_NAV_VELNED] = {.header = {.class = 0x01, .id = 0x12, .length = 0x00}, .checksum_a = 0x13, .checksum_b = 0x3a},
    [UBX_NAV_STAT] = {.header = {.class = 0x01, .id = 0x03, .length = 0x00}, .checksum_a = 0x04, .checksum_b = 0x0d},
    [UBX_MON_VER] = {.header = {.class = 0x0A, .id = 0x04, .length = 0x00}, .checksum_a = 0x0E, .checksum_b = 0x34},
    [UBX_NAV_PVT] = {.header = {.class = 0x01, .id = 0x07, .length = 0x00}, .checksum_a = 0x08, .checksum_b = 0x19},
};

/**
 * Gets the total length of a message, including checksums and sync characters
 * @return uint16_t
 */
static inline uint16_t ubx_message_length(const UBXFrame *msg) {
    // Includes two bytes for sync characters
    return sizeof(msg->header) + 2 + msg->header.length + sizeof(msg->checksum_a) + sizeof(msg->checksum_b);
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
 * Calculates a checksum on a UBX message that has an initialized header and payload, then returns that checksum in the
 * ck_a and ck_b locations, does not use checksum fields in the msg to calculate the checksum
 * @param msg The message to calculate the checksum for (using the header and payload)
 * @param ck_a The place to store the high byte of the checksum
 * @param ck_b The place to store the low byte of the checksum
 * @return errno_t
 */
static void calculate_checksum(UBXFrame *msg, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    calculate_partial_checksum((uint8_t *)(&msg->header), 4, ck_a, ck_b);
    // Payload not contiguous, so calculate the checksum in two halves
    calculate_partial_checksum((uint8_t *)msg->payload, msg->header.length, ck_a, ck_b);
}

/**
 * Tests if the checksum is valid
 * @param msg The message to check the checksum of, with an initialized header and payload
 * @return uint8_t Zero if the checksum is valid
 */
static inline uint8_t test_checksum(UBXFrame *msg) {
    uint8_t a, b;
    calculate_checksum(msg, &a, &b);
    return msg->checksum_a == a && msg->checksum_b == b;
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
 * @return int EINVAL if the message has no space for this configuration item, EOK otherwise
 */
static int add_valset_item(UBXFrame *msg, uint32_t key, const void *value, UBXValueType type) {
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
 * Returns the number of bytes ready to be read from the sensor. Currently does not exhibit the expected behaviour and
 * we don't know why, try this again later
 *
 * @param loc The m10spg's location on the I2C bus
 * @param result The total number of bytes ready to be read
 * @return int The error status of the call. EOK if successful.
 */
static int m10spg_available_bytes(const SensorLocation *loc, uint16_t *result) {
    // Send the address of the first register, then the second byte read will be the next register (0xFE)
    i2c_send_t write_header = {.stop = 0, .len = 1};
    write_header.slave = loc->addr;

    uint8_t address_cmd[sizeof(write_header) + 1];
    memcpy(address_cmd, &write_header, sizeof(write_header));
    address_cmd[sizeof(write_header)] = 0xFD;

    i2c_recv_t read_header = {.stop = 1, .len = 2};
    read_header.slave = loc->addr;
    uint8_t read_cmd[sizeof(read_header) + 2];
    memcpy(read_cmd, &read_header, sizeof(read_header));

    errno_t err = devctl(loc->bus, DCMD_I2C_SEND, address_cmd, sizeof(address_cmd), NULL);
    return_err(err);

    err = devctl(loc->bus, DCMD_I2C_RECV, read_cmd, sizeof(read_cmd), NULL);
    return_err(err);

    *result = ((uint16_t)read_cmd[sizeof(write_header)]) * 256 + (uint16_t)(read_cmd[sizeof(write_header) + 1]);
    return EOK;
}

/**
 * Reads a certain number of bytes into the specified buffer
 * @param loc The m10spg's location on the I2C bus
 * @param buf A pointer to the memory location to store the data.
 * @param nbytes The number of bytes to read into the buffer
 * @return int The error status of the call. EOK if successful.
 */
int read_bytes(const SensorLocation *loc, void *buf, size_t nbytes) {
    i2c_recv_t header = {.stop = 1};
    header.slave = loc->addr;
    header.len = nbytes;
    uint8_t read_cmd[sizeof(header) + nbytes];
    memcpy(read_cmd, &header, sizeof(header));

    errno_t err = devctl(loc->bus, DCMD_I2C_RECV, read_cmd, sizeof(read_cmd), NULL);
    if (err != EOK) return err;

    // Copy out what we recieved
    memcpy(buf, read_cmd + sizeof(header), nbytes);
    return EOK;
}

/**
 * Writes data to the M10SPG. Currently useless - UBX messages should be written using the send_ubx_message function
 * @param loc The m10spg's location on the I2C bus
 * @param buf A pointer to the memory location containing the data.
 * @param nbytes The number of bytes to be written to the M10SPG.
 * @return int Status of reading from the sensor, EOK if successful.
 */
static int write_bytes(const SensorLocation *loc, void *buf, size_t nbytes) {
    i2c_send_t header = {.stop = 1};
    header.slave = loc->addr;
    header.len = nbytes;
    uint8_t data[sizeof(header) + nbytes];
    memcpy(data, &header, sizeof(header));
    memcpy(data + sizeof(header), buf, nbytes);

    return devctl(loc->bus, DCMD_I2C_SEND, data, sizeof(header) + nbytes, NULL);
}

/**
 * Sends a UBX message
 * @param loc The m10spg's location on the I2C bus
 * @param msg A populated emssage structure, with its checksum already calculated
 * @return int Status of writing to the sensor, EOK if successful
 */
static int send_message(const SensorLocation *loc, const UBXFrame *msg) {
    i2c_send_t header = {.stop = 1};
    header.slave = loc->addr;
    header.len = ubx_message_length(msg);
    uint8_t data[sizeof(header) + ubx_message_length(msg)];
    memcpy(data, &header, sizeof(header));
    // Add sync characters
    data[sizeof(header)] = SYNC_ONE;
    data[sizeof(header) + 1] = SYNC_TWO;
    memcpy(data + 2 + sizeof(header), &msg->header, sizeof(msg->header));
    memcpy(data + 2 + sizeof(header) + sizeof(msg->header), msg->payload, msg->header.length);
    memcpy(data + 2 + sizeof(header) + sizeof(msg->header) + msg->header.length, &msg->checksum_a, 2);

    int err = devctl(loc->bus, DCMD_I2C_SEND, data, sizeof(data), NULL);
    return err;
}

/**
 * Gets the next ublox protcol message from the reciever, reading through any non-ublox data
 * @param loc The m10spg's location on the I2C bus
 * @param msg An empty message structure, with a payload pointing at a data buffer to read the payload into
 * @param max_payload The maximum size that the payload can be (the size of the buffer pointed to by it)
 * @return int EINVAL if the buffer is too small for the message found. ETIMEOUT if the timeout expires before a
 * message is found. EBADMSG if the second sync char is not valid. EOK otherwise.
 */
static int recv_message(const SensorLocation *loc, UBXFrame *msg, uint16_t max_payload) {
    uint8_t sync = 0;
    errno_t err = read_bytes(loc, &sync, sizeof(sync));
    return_err(err);
    // Make sure we're at the start of a new message
    if (sync == SYNC_ONE) {
        err = read_bytes(loc, &sync, sizeof(sync));
        return_err(err);
        if (sync == SYNC_TWO) {
            // Found something. Get message class, id, and length
            err = read_bytes(loc, &msg->header.class,
                             sizeof(msg->header.class) + sizeof(msg->header.id) + sizeof(msg->header.length));
            return_err(err);
            // Make sure the space we allocated for the payload is big enough
            if (msg->header.length > max_payload) {
                return EINVAL;
            }
            // Read payload
            err = read_bytes(loc, msg->payload, msg->header.length);
            return_err(err);
            // Read in the checksums (assume contiguous)
            err = read_bytes(loc, &msg->checksum_a, 2);
            return err;
        } else {
            return EBADMSG;
        }
    }
    return ENODATA;
}

/**
 * Reads messages from the sensor until a message of type msg_type is read
 * @param loc The m10spg's location on the I2C bus
 * @param msg_type The type of the last message to be read, if there are multiple messages in the data buffer
 * @param buf The location to store the message of msg_type. Buffer contents may be modified if a message is not found
 * @param size The size of the data buffer, should be at least large enough for any periodic messages that were registered
 * @return int 0 if the message of msg_type was placed in buf, ENODATA if the buffer is empty, and other errors for problems reading over I2C 
 */
int m10spg_read(const SensorLocation *loc, M10SPGMessageType msg_type, uint8_t *buf, size_t size) {
    UBXFrame recv;
    recv.payload = buf;
    int err = recv_message(loc, &recv, size);
    return err;
}

/**
 * Prepares the M10SPG for reading.
 * @param loc The m10spg's location on the I2C bus
 * @return int The error status of the call. EOK if successful.
 */
int m10spg_open(const SensorLocation *loc) {
    UBXFrame msg;
    UBXValsetPayload valset_payload;
    UBXAckPayload ack_payload;

    // Clear the RAM configuration to ensure our settings are the only ones being used
    msg.header.class = 0x06;
    msg.header.id = 0x04;
    msg.header.length = 4;

    UBXConfigResetPayload reset_payload;
    reset_payload.navBbrMask[0] = 0x00;
    reset_payload.navBbrMask[1] = 0x00;
    reset_payload.resetMode = UBX_SOFT_RESET;
    msg.payload = &reset_payload;
    calculate_checksum(&msg, &msg.checksum_a, &msg.checksum_b);
    send_message(loc, &msg);
    // Has no response, sleep to wait for reset
    sleep(1);

    // Configure the chip
    msg.payload = &valset_payload;

    // Put our actual configuration on there
    init_valset_message(&msg, RAM_LAYER);
    uint8_t config_disabled = 0;
    uint8_t config_dynmodel = DYNMODEL_AIR_4G;
    uint16_t measurement_rate = NOMINAL_MEASUREMENT_RATE;
    uint8_t config_enabled = 1;
    // Disable NMEA output on I2C
    add_valset_item(&msg, (uint32_t)NMEA_I2C_OUTPUT_CONFIG_KEY, &config_disabled, UBX_TYPE_L);
    // Disable NMEA input on I2C
    add_valset_item(&msg, (uint32_t)NMEA_I2C_INPUT_CONFIG_KEY, &config_disabled, UBX_TYPE_L);
    // Set the dynamic platform model to have the maximum speed, acceleration, and height possible
    add_valset_item(&msg, (uint32_t)DYNMODEL_CONFIG_KEY, &config_dynmodel, UBX_TYPE_U1);
    // Set the config update rate
    add_valset_item(&msg, (uint32_t)MEASUREMENT_RATE_CONFIG_KEY, &measurement_rate, UBX_TYPE_U2);
    // Turn off the BDS satellites, which increases the maximum update rate, but needs a reset of the GPS subsystem
    add_valset_item(&msg, (uint32_t)BSD_SIGNAL_CONFIG_KEY, &config_disabled, UBX_TYPE_L);
    // Enable periodic output of the NAV-PVT message (this doesn't change how often it is calculated, just output)
    add_valset_item(&msg, (uint32_t)MSGOUT_I2C_NAV_PVT, &config_enabled, UBX_TYPE_U1);

    calculate_checksum(&msg, &msg.checksum_a, &msg.checksum_b);

    int err = send_message(loc, &msg);
    return_err(err);

    // Check if configuration was successful
    msg.payload = &ack_payload;
    // Longer timeout for the reboot
    err = recv_message(loc, &msg, sizeof(ack_payload), DEFAULT_TIMEOUT);
    return_err(err);
    if (msg.header.class == 0x05) {
        if (msg.header.id == 0x01) {
            return EOK;
        } else if (msg.header.id == 0x00) {
            // Valset was not successful, check interface manual for possible reasons
            return ECANCELED;
        }
    }
    // Give at least 0.5 seconds for the gps subsystem to restart, because we disabled the BDS signal
    usleep(500000);

    // Some other response interrupted our exchange
    return EINTR;
}

/**
 * Helper function to sleep this thread until it's likely there will be a new payload in the data buffer soon
 * @param payload A payload, if we recieved one, or NULL if we did not or could not
 */
void wait_for_meas(UBXNavPVTPayload *payload) {
    // Garbage for now
    usleep(1000);
}

#ifdef __M10SPG_DEBUG__

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
 * @param loc The m10spg's location on the I2C bus
 * @param max_bytes The maximum number of bytes the message's payload can have
 * @param timeout The maximum time to wait for a message in the buffer
 * @return int The status of the read operation on the buffer, EOK if successful
 */
static int debug_print_next_ubx(const SensorLocation *loc, size_t max_bytes, size_t timeout) {
    uint8_t buffer[max_bytes];
    UBXFrame msg;
    msg.payload = buffer;
    errno_t err = recv_message(loc, &msg, max_bytes, timeout);
    return_err(err);
    debug_print_ubx_message(&msg);
    return EOK;
}

/**
 * A function for debugging the interface with the M10SPG module, prints a number of bytes from the I2C buffer,
 * displaying the bytes in hex and ascii. If the buffer is empty, a hex value of 0XFF is printed
 * @param loc The m10spg's location on the I2C bus
 * @param bytes The number of bytes to read from the buffer (can be greater than the number of bytes actually in the
 * buffer)
 * @return int The status of the read operation, EOK if successful
 */
static int debug_dump_buffer(const SensorLocation *loc, size_t bytes) {
    uint8_t buff[bytes];
    errno_t err = read_bytes(loc, buff, bytes);
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

#endif // __M10SPG_DEBUG__
