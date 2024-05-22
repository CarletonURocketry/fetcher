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

/** All the different registers available in the PAC195X. */
typedef enum {
    MANUFACTURER_ID = 0xFE, /** The register containing the manufacturer ID of 0x54. */
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
 * Reads a byte from the PAC195X, assuming the address pointer is already at the correct location.
 * @param loc The location of the sensor on the I2C bus.
 * @param data A pointer to where to store the byte just read.
 * @return Any error which occurred while communicating with the sensor. EOK if successful.
 */
static int pac195x_read_byte(SensorLocation const *loc, uint8_t *data) {

    i2c_recv_t hdr = {.len = 1, .stop = 1, .slave = loc->addr};
    uint8_t cmd[sizeof(hdr) + 1];
    memcpy(cmd, &hdr, sizeof(cmd));

    int err = devctl(loc->bus, DCMD_I2C_RECV, cmd, sizeof(cmd), NULL);
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
int pac195x_get_manu_id(SensorLocation const *loc, uint8_t *id) {
    int err = pac195x_send_byte(loc, MANUFACTURER_ID);
    return_err(err);
    err = pac195x_read_byte(loc, id);
    return err;
}
