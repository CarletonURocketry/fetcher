#include "eeprom.h"
#include <hw/i2c.h>
#include <string.h>

/** The address of the EEPROM on the I2C bus. */
#define EEPROM_ADDR 0x50

/** Address for reading the EEPROM. */
#define EEPROM_READ (EEPROM_ADDR | 0x1)

/** Address for writing to the EEPROM. */
#define EEPROM_WRITE (EEPROM_ADDR & 0xFE)

/** The maximum capacity of the EEPROM in bytes. */
#define EEPROM_CAP 128

/** Defines a small buffer for the dummy write request. */
struct dummy_write_t {
    i2c_send_t header;    /**< The send header containing address information. */
    uint8_t byte_address; /** The byte address for beginning the read. */
};

/**
 * Read `n` bytes from the EEPROM into a buffer.
 * @param addr The address to start the read from.
 * @param bus The I2C bus where the EEPROM is located.
 * @param buf A pointer to the buffer to store the data that will be read. Must have space for `n` + 20 bytes.
 * @param n The number of bytes to read from the EEPROM.
 */
errno_t eeprom_read(uint8_t addr, int bus, void *buf, size_t n) {

    // Dummy write to start from address
    static struct dummy_write_t dummy_write = {
        .header =
            {
                .stop = 0,
                .len = 1,
                .slave = {.fmt = I2C_ADDRFMT_7BIT, .addr = EEPROM_WRITE},
            },
    };
    dummy_write.byte_address = addr;
    errno_t err = devctl(bus, DCMD_I2C_SEND, &dummy_write, sizeof(dummy_write), NULL);
    if (err != EOK) return err;

    // Start sequential read into buffer
    i2c_sendrecv_t read_header = {
        .stop = 1,
        .send_len = 0,
        .recv_len = n,
        .slave = {.fmt = I2C_ADDRFMT_7BIT, .addr = EEPROM_WRITE},
    };
    memcpy(buf, &read_header, sizeof(read_header));
    err = devctl(bus, DCMD_I2C_SENDRECV, &((uint8_t *)(buf))[sizeof(read_header)], n, NULL);
    if (err != EOK) return err;
    return EOK;
}
