/**
 * @file m24c0x.c
 * @brief Driver utilities for the M24C0X family of EEPROMS.
 * Based on the datasheet: https://www.st.com/en/memories/m24c02-r.html
 */
#include "m24c0x.h"
#include <devctl.h>
#include <unistd.h>

/** Macro to early return error. */
#define return_err(err)                                                                                                \
    if ((err) != 0) return (err)

/**
 * Write a single byte to the EEPROM.
 * @param loc The location of the EEPROM on the I2C bus.
 * @param addr The address of the EEPROM to write the data to.
 * @param data The byte of data to write.
 * @return 0 if successful, the error that occurred otherwise.
 */
int m24c0x_write_byte(SensorLocation const *loc, uint8_t addr, uint8_t data) {
    // Temporary type for single byte payload
    struct {
        i2c_send_t hdr;
        uint8_t addr;
        uint8_t data;
    } cmd = {
        .hdr = {.len = 2, .stop = 1, .slave = loc->addr},
        .addr = addr,
        .data = data,
    };
    return devctl(loc->bus, DCMD_I2C_SEND, &cmd, sizeof(cmd), NULL);
}

/**
 * Write a page (16 bytes) of data to the EEPROM starting at `addr`.
 * WARNING: If the data is too long to fit in the page, the bytes exceeding the page end are written on the same page
 * from location 0.
 * @param loc The location of the EEPROM on the I2C bus.
 * @param addr The address of the EEPROM to write the data to.
 * @param data A buffer with data to write to the EEPROM.
 * @param nbytes The number of bytes in the data buffer to be written to the EEPROM.
 * @return 0 if successful, the error that occurred otherwise.
 */
int m24c0x_write_page(SensorLocation const *loc, uint8_t addr, uint8_t const *data, size_t nbytes) {

    if (nbytes > 16) return EINVAL; // Only one page at a time

    iov_t siov[2];
    struct {
        i2c_send_t hdr;
        uint8_t addr;
    } cmd = {.hdr = {.len = nbytes, .stop = 1, .slave = loc->addr}, .addr = addr};

    SETIOV(&siov[0], &cmd, sizeof(cmd));
    SETIOV(&siov[1], data, nbytes);
    return devctlv(loc->bus, DCMD_I2C_SEND, 2, 0, siov, NULL, NULL);
}

/**
 * Read a single byte from the current address.
 * @param loc The location of the EEPROM on the I2C bus.
 * @param data A buffer to store the byte that was read.
 * @return 0 if successful, the error that occurred otherwise.
 */
int m24c0x_read_cur_byte(SensorLocation const *loc, uint8_t *data) {

    // Temporary type for reading a single byte
    struct {
        i2c_recv_t hdr;
        uint8_t data;
    } cmd = {.hdr = {.len = 1, .stop = 1, .slave = loc->addr}};

    int err = devctl(loc->bus, DCMD_I2C_RECV, &cmd, sizeof(cmd), NULL);
    return_err(err);
    *data = cmd.data;
    return err;
}

/**
 * Read a single byte from a random address.
 * @param loc The location of the EEPROM on the I2C bus.
 * @param addr The address of the EEPROM to write the data to.
 * @param data The byte of data to write.
 * @return 0 if successful, the error that occurred otherwise.
 */
int m24c0x_read_rand_byte(SensorLocation const *loc, uint8_t addr, uint8_t *data) {

    // Temporary type for single byte payload
    struct {
        i2c_sendrecv_t hdr;
        uint8_t payload;
    } cmd = {
        .hdr = {.send_len = 1, .recv_len = 1, .stop = 1, .slave = loc->addr},
        .payload = addr,
    };
    int err = devctl(loc->bus, DCMD_I2C_SENDRECV, &cmd, sizeof(cmd), NULL);
    return_err(err);
    *data = cmd.payload;
    return err;
}

/**
 * Sequentially read `nbytes` of data from the current address location.
 * WARNING: If the read address goes past the EEPROM's maximum address, the read will continue starting from 0x00.
 * @param loc The location of the EEPROM on the I2C bus.
 * @param data A buffer with enough space for `nbytes` of data.
 * @param nbytes The number of bytes to read from the EEPROM into the data buffer.
 * @return 0 if successful, the error that occurred otherwise.
 */
int m24c0x_seq_read_cur(SensorLocation const *loc, uint8_t *data, size_t nbytes) {
    i2c_recv_t hdr = {.len = nbytes, .stop = 1, .slave = loc->addr};

    // IO vector for sending and receiving
    iov_t siov[2];
    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], data, nbytes);

    return devctlv(loc->bus, DCMD_I2C_RECV, 2, 2, siov, siov, NULL);
}

/**
 * Sequentially read `nbytes` of data starting from `addr`.
 * WARNING: If the read address goes past the EEPROM's maximum address, the read will continue starting from 0x00.
 * @param loc The location of the EEPROM on the I2C bus.
 * @param addr The address of the EEPROM to start the read from.
 * @param data A buffer with enough space for `nbytes` of data.
 * @param nbytes The number of bytes to read from the EEPROM into the data buffer.
 * @return 0 if successful, the error that occurred otherwise.
 */
int m24c0x_seq_read_rand(SensorLocation const *loc, uint8_t addr, uint8_t *data, size_t nbytes) {

    i2c_sendrecv_t hdr = {.send_len = 1, .recv_len = nbytes, .stop = 1, .slave = loc->addr};
    data[0] = addr; // First byte contains address for the send part of the command

    // IO vector for sending and receiving
    iov_t siov[2];
    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], data, nbytes);

    return devctlv(loc->bus, DCMD_I2C_SENDRECV, 2, 2, siov, siov, NULL);
}

/**
 * Write a value of '0' to all the addresses in the EEPROM, thereby erasing it.
 * @param loc The location of the EEPROM on the I2C bus.
 * @param size The capacity of the EEPROM in bytes.
 * @return 0 if successful, the error that occurred otherwise.
 */
int m24c0x_erase(SensorLocation const *loc, size_t size) {
    int err;
    for (uint16_t addr = 0; addr < size; addr++) {
        err = m24c0x_write_byte(loc, addr, 0);
        return_err(err);
    }
    return 0;
}
