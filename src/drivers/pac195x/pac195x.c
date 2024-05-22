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
    REVISION_ID = 0xFE,         /**< The register containing the revision ID. Initial release is 0x02. */
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
    memcpy(cmd, &hdr, sizeof(cmd));
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
    i2c_send_t hdr = {.len = 1, .stop = 1, .slave = loc->addr};
    uint8_t cmd[sizeof(hdr) + 2];
    memcpy(cmd, &hdr, sizeof(cmd));
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
    memcpy(cmd, &hdr, sizeof(cmd));

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
    memcpy(cmd, &hdr, sizeof(cmd));
    cmd[sizeof(hdr)] = addr;

    int err = devctl(loc->bus, DCMD_I2C_SENDRECV, cmd, sizeof(cmd), NULL);
    return_err(err);
    *data = cmd[sizeof(hdr)];
    return err;
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
 * Reads the revision ID from the PAC195X into `id`.
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
