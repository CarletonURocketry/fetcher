#include "board_id.h"
#include <stdlib.h>

/**
 * Reads the sensor name from the board ID into a new buffer.
 * @param board_id The contents of the board ID.
 * @param sensor_name The buffer to store the sensor name.
 * @param nbytes The maximum number of bytes to read.
 * @return The current position inside the board ID after reading, or NULL if nbytes needs to be larger.
 */
const char *read_sensor_name(const char *board_id, char *sensor_name, uint8_t nbytes) {
    uint8_t pos;
    for (pos = 0; *board_id != ' ' && *board_id != '\0' && pos < nbytes; pos++, board_id++) {
        sensor_name[pos] = *board_id;
    }

    if (pos == nbytes) return NULL; // Could not successfully read the contents

    // We didn't hit end of content, skip newline character
    if (*board_id != '\0') board_id++;

    sensor_name[pos] = '\0';
    return board_id;
}

/**
 * Reads up to `naddrs` addresses from the board ID into an array of `addresses`.
 * @param board_id The current position in the board ID where an address starts.
 * @param addresses An array of bytes with enough space to store `naddrs` bytes.
 * @param naddrs A pointer to the maximum number of addresses to read. After the function call, this pointer will
 * contain the actual number of addresses read.
 * @return The current position in the board ID after the read, or NULL if `naddrs` needs to be larger.
 */
const char *read_sensor_addresses(const char *board_id, uint8_t *addresses, uint8_t *naddrs) {

    uint8_t num_addrs;
    for (num_addrs = 0; *board_id != '\n' && *board_id != '\0' && num_addrs < *naddrs; board_id++) {
        if (*board_id == ' ') {
            // Address is hex with two digits, so two digits back from space will be address start
            // Convert this hex into an actual numerical byte
            addresses[num_addrs] = strtoul(board_id - 2, NULL, 16);
            num_addrs++;
        }
    }

    if (num_addrs == *naddrs) return NULL; // Need to be able to read more addresses

    // We hit end of content
    if (*board_id == '\0') {
        *naddrs = num_addrs;
        return board_id;
    }

    // Hit a newline, get last address (three positions back because board_id was incremented an extra time exiting the
    // for loop)
    addresses[num_addrs] = strtoul(board_id - 3, NULL, 16);
    num_addrs++;
    board_id++; // Skip over final newline character

    *naddrs = num_addrs;
    return board_id;
}
