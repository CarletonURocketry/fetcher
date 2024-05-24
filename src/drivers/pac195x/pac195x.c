/**
 * @file pac195x.c
 * @brief Includes a driver for the PAC195X family of power monitors. Currently tested on the PAC1952.
 * Datasheet:
 * https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/DataSheets/PAC195X-Family-Data-Sheet-DS20006539.pdf
 */

#include "pac195x.h"
#include "sensor_api.h"
#include <errno.h>
#include <hw/i2c.h>
#include <stdint.h>
#include <string.h>

/** Macro to early return error statuses. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** All the different registers/commands available in the PAC195X. */
typedef enum {
    REFRESH = 0x00,     /**< Refreshes the PAC195X. */
    CTRL = 0x01,        /**< Control register for configuring sampling mode and ALERT pins. */
    ACC_COUNT = 0x02,   /**< Accumulator count for all channels. */
    VACCN = 0x03,       /**< Accumulator outputs for channels 1-4. Spans until 0x06. */
    VBUSN = 0x07,       /**< V_BUS measurements for channels 1-4. Spans until 0x0A. */
    VSENSEN = 0x0B,     /**< V_SENSE measurements for channels 1-4. Spans until 0x0E. */
    VBUSN_AVG = 0x0F,   /**< Rolling average of the 8 most recent V_BUS_n measurements, (n: 1-4). Spans until 0x12. */
    VSENSEN_AVG = 0x13, /**< Rolling average of the 8 most recent V_SENSE_n measurements, (n: 1-4). Spans until 0x16. */
    VPOWERN = 0x17,     /**< V_SENSE * V_BUS for channels 1-4. Spans until 0x1A. */
    SMBUS_SETTINGS = 0x1C,      /**< Activate SMBus functionality, I/O data for R/W on I/O pins. */
    NEG_PWR_FSR = 0x1D,         /**< Configuration control for bidirectional current. */
    REFRESH_G = 0x1E,           /**< Refresh response to General Call Adddress. */
    REFRESH_V = 0x1F,           /**< Refreshes V_BUS and V_SENSE data only, no accumulator reset. */
    SLOW = 0x20,                /**< Status and control for SLOW pin functions. */
    CTRL_ACT = 0x21,            /**< Currently active value of CTRL register. */
    NEG_PWR_FSR_ACT = 0x22,     /**< Currently active value of NEG_PWR register. */
    CTRL_LAT = 0x23,            /**< Latched active value of CTRL register. */
    NWG_PWR_FSR_LAT = 0x24,     /**< Latched active value of NEG_PWR register. */
    ACCUM_CONFIG = 0x25,        /**< Enable V_SENSE and V_BUS accumulation. */
    ALERT_STATUS = 0x26,        /**< Reads to see what triggered ALERT. */
    SLOW_ALERT1 = 0x27,         /**< Assigns specific ALERT to ALERTn/SLOW. */
    GPIO_ALERT2 = 0x28,         /**< Assigns specific ALERT to ALERTn/I/O. */
    ACC_FULLNESS_LIMITS = 0x29, /**< ACC and ACC Count fullness limits. */
    OC_LIMITN = 0x30,           /**< OC limit for channels 1-4. Spans until 0x33. */
    UC_LIMITN = 0x34,           /**< UC limit for channels 1-4. Spans until 0x37. */
    OP_LIMITN = 0x38,           /**< OP limit for channels 1-4. Spans until 0x3B. */
    OV_LIMITN = 0x3C,           /**< OV limit for channels 1-4. Spans until 0x3F. */
    UV_LIMITN = 0x40,           /**< UV limit for channels 1-4. Spans until 0x43. */
    OC_LIMIT_NSAMPLES = 0x44,   /**< Consecutive OC samples over threshold for ALERT. */
    UC_LIMIT_NSAMPLES = 0x45,   /**< Consecutive UC samples over threshold for ALERT. */
    OP_LIMIT_NSAMPLES = 0x46,   /**< Consecutive OP samples over threshold for ALERT. */
    OV_LIMIT_NSAMPLES = 0x47,   /**< Consecutive OV samples over threshold for ALERT. */
    UV_LIMIT_NSAMPLES = 0x48,   /**< Consecutive UV samples over threshold for ALERT. */
    ALERT_ENABLE = 0x49,        /**< ALERT enable. */
    ACCUM_CONFIG_ACT = 0x50,    /**< Currently active value of ACCUM_CONFIG register. */
    ACCUM_CONFIG_LAT = 0x51,    /**< Currently latched value of ACCUM_CONFIG register. */
    PRODUCT_ID = 0xFD,          /**< The register containing the product ID. */
    MANUFACTURER_ID = 0xFE,     /**< The register containing the manufacturer ID of 0x54. */
    REVISION_ID = 0xFF,         /**< The register containing the revision ID. Initial release is 0x02. */
} pac195x_reg_t;

/**
 * Set the internal address pointer of the PAC195X to the specified address (in preparation for write/read).
 * @param loc The location of the sensor on the I2C bus.
 * @param addr The register address to set the address pointer to.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
static int pac195x_send_byte(SensorLocation const *loc, uint8_t addr) {
    i2c_send_t hdr = {.len = 1, .stop = 1, .slave = loc->addr};
    uint8_t cmd[sizeof(hdr) + 1];
    memcpy(cmd, &hdr, sizeof(hdr));
    cmd[sizeof(hdr)] = addr;

    return devctl(loc->bus, DCMD_I2C_SEND, cmd, sizeof(cmd), NULL);
}

/**
 * Writes the byte to the specified register of the PAC195X.
 * @param loc The location of the sensor on the I2C bus.
 * @param addr The register address to write to.
 * @param data The data to write.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
static int pac195x_write_byte(SensorLocation const *loc, uint8_t addr, uint8_t data) {
    i2c_send_t hdr = {.len = 2, .stop = 1, .slave = loc->addr};
    uint8_t cmd[sizeof(hdr) + 2];
    memcpy(cmd, &hdr, sizeof(hdr));
    cmd[sizeof(hdr)] = addr;
    cmd[sizeof(hdr) + 1] = data;

    return devctl(loc->bus, DCMD_I2C_SEND, cmd, sizeof(cmd), NULL);
}

/**
 * Reads a byte from the PAC195X, assuming the address pointer is already at the correct location.
 * @param loc The location of the sensor on the I2C bus.
 * @param data A pointer to where to store the byte just read.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
static int pac195x_receive_byte(SensorLocation const *loc, uint8_t *data) {
    i2c_recv_t hdr = {.len = 1, .stop = 1, .slave = loc->addr};
    uint8_t cmd[sizeof(hdr) + 1];
    memcpy(cmd, &hdr, sizeof(hdr));

    int err = devctl(loc->bus, DCMD_I2C_RECV, cmd, sizeof(cmd), NULL);
    return_err(err);
    *data = cmd[sizeof(hdr)];
    return err;
}

/**
 * Read a byte from a specific register address.
 * @param loc The location of the sensor on the I2C bus.
 * @param addr The register address to read from.
 * @param data A pointer to where to store the byte just read.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
static int pac195x_read_byte(SensorLocation const *loc, uint8_t addr, uint8_t *data) {
    i2c_sendrecv_t hdr = {.send_len = 1, .recv_len = 1, .stop = 1, .slave = loc->addr};
    uint8_t cmd[sizeof(hdr) + 1];
    memcpy(cmd, &hdr, sizeof(hdr));
    cmd[sizeof(hdr)] = addr;

    int err = devctl(loc->bus, DCMD_I2C_SENDRECV, cmd, sizeof(cmd), NULL);
    return_err(err);
    *data = cmd[sizeof(hdr)];
    return err;
}

/**
 * Read several bytes from the PAC195X starting at a specific address. After the call, data will be stored in `buf[20]`
 * and onward (inclusive).
 * @param loc The location of the sensor on the I2C bus.
 * @param addr The register address to read from.
 * @param nbytes The number of bytes to read. Cannot be 0.
 * @param buf A pointer to where to store the bytes just read. Must have room for `nbytes + 20`.
 * @return Any error which occurred while communicating with the sensor. EOK if successful, EINVAL if nbytes is 0.
 */
static int pac195x_block_read(SensorLocation const *loc, uint8_t addr, size_t nbytes, uint8_t *buf) {

    if (nbytes == 0) return EINVAL;

    i2c_sendrecv_t hdr = {.send_len = 1, .recv_len = nbytes, .stop = 1, .slave = loc->addr};
    memcpy(buf, &hdr, sizeof(hdr));
    buf[sizeof(hdr)] = addr;

    return devctl(loc->bus, DCMD_I2C_SENDRECV, buf, nbytes + sizeof(hdr), NULL);
}

/**
 * Write several bytes to the PAC195X starting at a specific address.
 * @param loc The location of the sensor on the I2C bus.
 * @param addr The register address to write to.
 * @param nbytes The number of bytes to write. Cannot be 0.
 * @param buf A pointer to where the data to be written is located. Data must be preceded with 17 bytes of empty space.
 * @return Any error which occurred while communicating with the sensor. EOK if successful, EINVAL if nbytes is 0.
 */
static int pac195x_block_write(SensorLocation const *loc, uint8_t addr, size_t nbytes, uint8_t *buf) {

    if (nbytes == 0) return EINVAL;

    i2c_send_t hdr = {.len = nbytes + 1, .stop = 1, .slave = loc->addr};
    memcpy(buf, &hdr, sizeof(hdr));
    buf[sizeof(hdr)] = addr; // Immediately after this addr is where the caller should have put their data.

    return devctl(loc->bus, DCMD_I2C_SEND, buf, nbytes + sizeof(hdr) + 1, NULL);
}

/**
 * Reads the manufacturer ID from the PAC195X into `id`. Should always be 0x54.
 * @param loc The location of the sensor on the I2C bus.
 * @param id A pointer to where the ID returned by the sensor will be stored.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
int pac195x_get_manu_id(SensorLocation const *loc, uint8_t *id) { return pac195x_read_byte(loc, MANUFACTURER_ID, id); }

/**
 * Reads the product ID from the PAC195X into `id`. The value is chip model dependent.
 * @param loc The location of the sensor on the I2C bus.
 * @param id A pointer to where the ID returned by the sensor will be stored.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
int pac195x_get_prod_id(SensorLocation const *loc, uint8_t *id) { return pac195x_read_byte(loc, PRODUCT_ID, id); }

/**
 * Reads the revision ID from the PAC195X into `id`. Should be 0x02.
 * @param loc The location of the sensor on the I2C bus.
 * @param id A pointer to where the ID returned by the sensor will be stored.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
int pac195x_get_rev_id(SensorLocation const *loc, uint8_t *id) { return pac195x_read_byte(loc, REVISION_ID, id); }

/**
 * Sends the refresh command to the PAC195X sensor.
 * @param loc The location of the sensor on the I2C bus.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
int pac195x_refresh(SensorLocation const *loc) { return pac195x_send_byte(loc, REFRESH); }

/**
 * Sends the refresh general call command to the PAC195X sensor.
 * WARNING: This command is received by all I2C slave devices. They will interpret this as a software reset if
 * implemented. This command may have unintended consequences.
 * @param loc The location of the sensor on the I2C bus.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
int pac195x_refresh_g(SensorLocation const *loc) {
    SensorLocation general_loc = {
        .bus = loc->bus,
        .addr = loc->addr,
    };
    general_loc.addr.addr = 0x0;
    return pac195x_send_byte(&general_loc, REFRESH_G);
}

/**
 * Sends the refresh v command to the PAC195X. Same as refresh except accumulators and accumulator count are not reset.
 * @param loc The location of the sensor on the I2C bus.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
int pac195x_refresh_v(SensorLocation const *loc) { return pac195x_send_byte(loc, REFRESH_V); }

/**
 * Set the sampling mode for the PAC195x.
 * @param loc The location of the sensor on the I2C bus.
 * @param mode The sampling mode to set.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
int pac195x_set_sample_mode(SensorLocation const *loc, pac195x_sm_e mode) {
    uint8_t ctrl_reg;
    int err = pac195x_read_byte(loc, CTRL, &ctrl_reg);
    return_err(err);

    ctrl_reg &= ~(0xF0); // Clear upper 4 bits
    ctrl_reg |= mode;    // Set the mode
    err = pac195x_write_byte(loc, CTRL, ctrl_reg);
    return err;
}

/**
 * Enable/disable channels for sampling on the PAC195X.
 * NOTE: Some models, like the PAC1952-2, physically do not have all channel pins. Trying to enable those channels will
 * do nothing.
 * @param loc The location of the sensor on the I2C bus.
 * @param channel A channel or multiple channels to enable/disable. Multiple channels can be given by ORing the channels
 * together.
 * @param enable True to enable the channel(s), false to disable the channel(s).
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
int pac195x_toggle_channel(SensorLocation const *loc, pac195x_channel_e channel, bool enable) {

    uint8_t buf[sizeof(i2c_sendrecv_t) + 2];
    int err = pac195x_block_read(loc, CTRL, 2, buf);
    return_err(err);
    if (enable) {
        buf[sizeof(i2c_sendrecv_t) + 1] &= ~(channel << 4); // Set the channels on (0 enables)
    } else {
        buf[sizeof(i2c_sendrecv_t) + 1] |= (channel << 4); // Set the channels off (1 disables)
    }
    buf[sizeof(i2c_send_t) + 1] = buf[sizeof(i2c_sendrecv_t)]; // Move back because write header takes less space
    buf[sizeof(i2c_send_t) + 2] = buf[sizeof(i2c_sendrecv_t) + 1];
    err = pac195x_block_write(loc, CTRL, 2, buf);
    return err;
}

/**
 * Generic function for reading 2 bit values from a specific channel number.
 * @param loc The location of the sensor on the I2C bus.
 * @param n The channel number (1-4, inclusive) to get the measurement from.
 * @param addr A pointer to where to store the value.
 * @param val A pointer to where to store the value.
 * @return Any error which occurred while communicating with the sensor. EOK if successful. EINVAL if `n` is an invalid
 * channel number.
 */
static int pac195x_get_16b_channel(SensorLocation const *loc, uint8_t addr, uint8_t n, uint16_t *val) {
    if (n > 4 || n < 1) return EINVAL; // Invalid channel number

    uint8_t buf[sizeof(i2c_sendrecv_t) + 2]; // Space for header and 16 bit response.
    int err = pac195x_block_read(loc, addr + (n - 1), 2, buf);
    return_err(err);
    *val = 0;
    *val |= (uint32_t)(buf[sizeof(i2c_sendrecv_t)] << 8);
    *val |= (uint32_t)buf[sizeof(i2c_sendrecv_t) + 1];
    return err;
}

/**
 * Get the V_SENSE measurements for channels 1-4.
 * NOTE: If SKIP is enabled and the caller attempts to read from a channel that is disabled, an I/O error will be
 * returned.
 * @param loc The location of the sensor on the I2C bus.
 * @param n The channel number (1-4, inclusive) to get the measurement from.
 * @param val A pointer to where to store the value.
 * @return Any error which occurred while communicating with the sensor. EOK if successful. EINVAL if `n` is an invalid
 * channel number.
 */
int pac195x_get_vsensen(SensorLocation const *loc, uint8_t n, uint16_t *val) {
    return pac195x_get_16b_channel(loc, VSENSEN, n, val);
}

/**
 * Get the V_BUS measurements for channels 1-4.
 * NOTE: If SKIP is enabled and the caller attempts to read from a channel that is disabled, an I/O error will be
 * returned.
 * @param loc The location of the sensor on the I2C bus.
 * @param n The channel number (1-4, inclusive) to get the measurement from.
 * @param val A pointer to where to store the value.
 * @return Any error which occurred while communicating with the sensor. EOK if successful. EINVAL if `n` is an invalid
 * channel number.
 */
int pac195x_get_vbusn(SensorLocation const *loc, uint8_t n, uint16_t *val) {
    return pac195x_get_16b_channel(loc, VBUSN, n, val);
}

/**
 * Get the V_BUS_AVG measurements for channels 1-4.
 * NOTE: If SKIP is enabled and the caller attempts to read from a channel that is disabled, an I/O error will be
 * returned.
 * @param loc The location of the sensor on the I2C bus.
 * @param n The channel number (1-4, inclusive) to get the measurement from.
 * @param val A pointer to where to store the value.
 * @return Any error which occurred while communicating with the sensor. EOK if successful. EINVAL if `n` is an invalid
 * channel number.
 */
int pac195x_get_vbusnavg(SensorLocation const *loc, uint8_t n, uint16_t *val) {
    return pac195x_get_16b_channel(loc, VBUSN_AVG, n, val);
}

/**
 * Get the V_SENSE_AVG measurements for channels 1-4.
 * NOTE: If SKIP is enabled and the caller attempts to read from a channel that is disabled, an I/O error will be
 * returned.
 * @param loc The location of the sensor on the I2C bus.
 * @param n The channel number (1-4, inclusive) to get the measurement from.
 * @param val A pointer to where to store the value.
 * @return Any error which occurred while communicating with the sensor. EOK if successful. EINVAL if `n` is an invalid
 * channel number.
 */
int pac195x_get_vsensenavg(SensorLocation const *loc, uint8_t n, uint16_t *val) {
    return pac195x_get_16b_channel(loc, VSENSEN_AVG, n, val);
}

/**
 * Get the V_POWER measurements for channels 1-4.
 * NOTE: If SKIP is enabled and the caller attempts to read from a channel that is disabled, an I/O error will be
 * returned.
 * @param loc The location of the sensor on the I2C bus.
 * @param n The channel number (1-4, inclusive) to get the measurement from.
 * @param val A pointer to where to store the value.
 * @return Any error which occurred while communicating with the sensor. EOK if successful. EINVAL if `n` is an invalid
 * channel number.
 */
int pac195x_get_powern(SensorLocation const *loc, uint8_t n, uint32_t *val) {
    if (n > 4 || n < 1) return EINVAL; // Invalid channel number

    uint8_t buf[sizeof(i2c_sendrecv_t) + 4]; // Space for header and 32 bit response.
    int err = pac195x_block_read(loc, VPOWERN + (n - 1), 4, buf);
    return_err(err);
    *val = *(uint32_t *)(&buf[sizeof(i2c_sendrecv_t)]); // TODO: fix byte ordering
    return err;
}

/**
 * Get the V_ACCN measurements for channels 1-4.
 * NOTE: If SKIP is enabled and the caller attempts to read from a channel that is disabled, an I/O error will be
 * returned.
 * @param loc The location of the sensor on the I2C bus.
 * @param n The channel number (1-4, inclusive) to get the measurement from.
 * @param val A pointer to where to store the value.
 * @return Any error which occurred while communicating with the sensor. EOK if successful. EINVAL if `n` is an invalid
 * channel number.
 */
int pac195x_get_vaccn(SensorLocation const *loc, uint8_t n, uintptr64_t *val) {
    if (n > 4 || n < 1) return EINVAL; // Invalid channel number

    uint8_t buf[sizeof(i2c_sendrecv_t) + 8] = {0};              // Space for header and 64 bit response.
    int err = pac195x_block_read(loc, VACCN + (n - 1), 7, buf); // Only as the 7 bytes within the VACCN register
    return_err(err);
    *val = *(uint32_t *)(&buf[sizeof(i2c_sendrecv_t)]); // TODO: fix byte ordering
    return err;
}

/**
 * Calculates the voltage on the SENSE line from the VBUS measurement.
 * @param fsr The full scale range to use for the calculation (PAC195X uses a default of 32).
 * @param vbus The measured VBUS channel value corresponding to the SENSE line.
 * @param bipolar Whether the measurement is bipolar or not (PAC195X uses unipolar by default).
 * @return The voltage measurement on the line in millivolts.
 */
uint32_t pac195x_calc_bus_voltage(uint8_t fsr, uint16_t vbus, bool bipolar) {
    uint16_t denominator;
    if (bipolar) {
        denominator = 32768;
    } else {
        denominator = 65535; // Actual calculation says to use 65536, but this approximation saves 2 bytes
    }
    return (fsr * vbus * 1000) / denominator;
}

/**
 * Calculate the bus current.
 * @param rsense The value of the R_SENSE resistor connected to the SENSE line in milliohms.
 * @param vsense The measured VSENSE channel value corresponding to the SENSE line.
 * @param bipolar Whether the measurement is bipolar or not (PAC195X uses unipolar by default).
 * @return The bus current in milliamps.
 */
uint32_t pac195x_calc_bus_current(uint32_t rsense, uint16_t vsense, bool bipolar) {
    uint16_t denominator;
    if (bipolar) {
        denominator = 32768;
    } else {
        denominator = 65535; // Actual calculation says to use 65536, but this approximation saves 2 bytes
    }
    return (100 * vsense * 1000) / (denominator * rsense);
}
