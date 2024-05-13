/**
 * @file ms5611.c
 * @brief Sensor API interface implementation for the MS5611 pressure and temperature sensor.
 *
 * Sensor API interface implementation for the MS5611 pressure and temperature sensor which uses I2C communication.
 *
 * See
 * https://www.te.com/commerce/DocumentDelivery/DDEController?Action=showdoc&DocId=Data+Sheet%7FMS5611-01BA03%7FB3%7Fpdf%7FEnglish%7FENG_DS_MS5611-01BA03_B3.pdf%7FCAT-BLPS0036.
 * for the MS5611 data sheet:
 *
 * See
 * https://www.amsys-sensor.com/downloads/notes/ms5611-precise-altitude-measurement-with-a-pressure-sensor-module-amsys-509e.pdf
 * for information about the MS5611 altitude measurement calculations.
 */
#include "ms5611.h"
#include "../sensor_api.h"
#include <errno.h>
#include <hw/i2c.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) goto return_defer

/** Defines the universal gas constant. */
#define R 8.31432

/** Defines constant for acceleration due to gravity. */
#define g 9.80665

/** Defines constant for the mean molar mass of atmospheric gases. */
#define M 0.0289644

/** Defines constant for the absolute temperature in Kelvins. */
#define T 273

/** All of the I2C commands that can be used on the MS5611 sensor. */
typedef enum {
    CMD_RESET = 0x1E,    /**< ADC reset command */
    CMD_PROM_RD = 0xA0,  /**< Prom read command */
    CMD_ADC_CONV = 0x40, /**< ADC conversion command */
    CMD_ADC_READ = 0x00, /**< ADC read command */
} MS5611Cmd;

/** The D registers on the MS5611 sensor. */
typedef enum {
    D1 = 0x00, /**< D register 1 */
    D2 = 0x10, /**< D register 2 */
} MS5611DReg;

/**
 * Resets the MS5611 sensor, ensuring that calibration data is loaded into the PROM.
 * @param loc The sensor's location on the I2C bus.
 * @return Any error from the attempted reset. EOK if successful.
 */
errno_t ms5611_reset(SensorLocation *loc) {
    i2c_send_t reset = {.stop = 1, .len = 1, .slave = loc->addr};
    uint8_t reset_cmd[sizeof(i2c_send_t) + 1];
    memcpy(reset_cmd, &reset, sizeof(reset));
    reset_cmd[sizeof(reset)] = CMD_RESET;

    errno_t err = devctl(loc->bus, DCMD_I2C_LOCK, NULL, 0, NULL); // Lock I2C bus
    return_err(err);

    err = devctl(loc->bus, DCMD_I2C_SEND, &reset_cmd, sizeof(reset_cmd), NULL);

return_defer:
    devctl(loc->bus, DCMD_I2C_UNLOCK, NULL, 0, NULL); // Unlock I2C bus
    return err;
}

/**
 * Reads the ADC D registers of the MS5611.
 * @param loc The location of the MS5611 on the I2C bus.
 * @param dreg The D-register to read and the precision to read at.
 * @param value The location to store the D register value in.
 * @return Any error which occurred during the read. EOK if successful.
 */
static errno_t ms5611_read_dreg(SensorLocation *loc, uint8_t dreg, uint32_t *value) {

    // Request conversion start
    i2c_send_t conversion = {.len = 1, .stop = 1, .slave = loc->addr};
    uint8_t conversion_cmd[sizeof(conversion) + 1];
    memcpy(conversion_cmd, &conversion, sizeof(conversion));
    conversion_cmd[sizeof(conversion)] = CMD_ADC_CONV + dreg;

    // Lock I2C bus before sending command
    errno_t err = devctl(loc->bus, DCMD_I2C_LOCK, NULL, 0, NULL);
    if (err != EOK) return err;

    err = devctl(loc->bus, DCMD_I2C_SEND, &conversion_cmd, sizeof(conversion_cmd), NULL);
    if (err != EOK) {
        devctl(loc->bus, DCMD_I2C_UNLOCK, NULL, 0, NULL);
        return err;
    }

    // Wait for appropriate conversion time
    devctl(loc->bus, DCMD_I2C_UNLOCK, NULL, 0, NULL);
    switch (dreg & 0xF) {
    case ADC_RES_256:
        usleep(900);
        break;
    case ADC_RES_512:
        usleep(3000);
        break;
    case ADC_RES_1024:
        usleep(4000);
        break;
    case ADC_RES_2048:
        usleep(6000);
        break;
    case ADC_RES_4096:
        usleep(10000);
        break;
    }

    // Re-lock bus after waiting
    err = devctl(loc->bus, DCMD_I2C_LOCK, NULL, 0, NULL);
    if (err != EOK) return err;

    // Read D register
    i2c_sendrecv_t read = {.slave = loc->addr, .stop = 1, .send_len = 1, .recv_len = 3};
    uint8_t read_cmd[sizeof(read) + 3];
    memcpy(read_cmd, &read, sizeof(read));
    read_cmd[sizeof(read)] = CMD_ADC_READ;
    err = devctl(loc->bus, DCMD_I2C_SENDRECV, &read_cmd, sizeof(read_cmd), NULL);

    if (err != EOK) {
        devctl(loc->bus, DCMD_I2C_UNLOCK, NULL, 0, NULL);
        return err;
    }

    // Unlock I2C bus
    err = devctl(loc->bus, DCMD_I2C_UNLOCK, NULL, 0, NULL);

    *value = 0;
    *value += read_cmd[sizeof(read)] * 65536;
    *value += read_cmd[sizeof(read) + 1] * 256;
    *value += read_cmd[sizeof(read) + 2];

    return err;
}

/**
 * Compute the double precision calculation.
 * @param dt The temperature delta.
 * @param off A pointer to the offset value.
 * @param sens A pointer to the sensitivity value.
 * @param temp A pointer to the current temperature value in millidegrees Celsius.
 */
static void double_precision(double dt, double *off, double *sens, double *temp) {
    double t2 = 0;
    double off2 = 0;
    double sens2 = 0;

    if (*temp < 20) {
        t2 = (dt * dt) / pow(2, 31);
        double temp_square = pow((*temp - 2000), 2);
        off2 = (5 * temp_square) / 2;
        sens2 = (5 * temp_square) / 4;

        if (*temp < -15) {
            temp_square = pow((*temp + 1500), 2);
            off2 = off2 + 7 * temp_square;
            sens2 = sens2 + 11 * temp_square / 2;
        }
    }

    *temp -= t2;
    *off -= off2;
    *sens -= sens2;
}

/**
 * Reads the specified data from the MS5611.
 * @param loc The location of the MS5611 sensor on the I2C bus.
 * @param res The resolution to read the MS5611 at.
 * @param ctx The context containing calibration coefficients and ground pressure.
 * @param precise True to use second order calculation for higher precision, false for quicker calculation.
 * @param temperature Storage location of the temperature value in degrees Celsius. NULL to skip calculation.
 * @param pressure Storage location of the pressure value in kPa. NULL to skip calculation.
 * @param altitude Storage location of the altitude value in m. NULL to skip calculation.
 * @return EOK if no error, otherwise the type of error that occurred.
 */
errno_t ms5611_read_all(SensorLocation *loc, MS5611Resolution res, MS5611Context *ctx, bool precise,
                        double *temperature, double *pressure, double *altitude) {

    // Variables for calculation
    double temp;
    double pres;

    // Read D registers with configured resolution
    uint32_t d1, d2;
    errno_t err = ms5611_read_dreg(loc, D1 + res, &d1);
    if (err != EOK) return err;
    err = ms5611_read_dreg(loc, D2 + res, &d2);
    if (err != EOK) return err;

    // Extract context
    // Calculate 1st order pressure and temperature (MS5607 1st order algorithm)
    double dt = d2 - ctx->coefs[5] * pow(2, 8);
    double off = ctx->coefs[2] * pow(2, 16) + dt * ctx->coefs[4] / pow(2, 7);
    double sens = ctx->coefs[1] * pow(2, 15) + dt * ctx->coefs[3] / pow(2, 8);

    temp = (2000 + ((dt * ctx->coefs[6]) / pow(2, 23)));

    // Second order algorithm only if high precision was chosen
    if (precise) double_precision(dt, &off, &sens, &temp);

    // Store temperature if requested
    if (temperature != NULL) {
        *temperature = temp / 100; // Convert from millidegrees Celsius to degrees Celsius
    }

    // Calculate pressure unless it's not needed for any calculation
    if (pressure != NULL || altitude != NULL) {
        pres = (((d1 * sens) / (pow(2, 21)) - off) / pow(2, 15)) / 1000; // kPa
        *pressure = pres;
    }

    // This calculation assumes initial altitude is 0
    if (altitude != NULL) {
        *altitude = -((R * T) / (g * M)) * log(pres / ctx->ground_pressure);
    }

    return err;
}

/**
 * Initialize the coefficients of the MS5611.
 * @param loc The location of the MS5611 sensor.
 * @param ctx The context struct to store the coefficients in.
 * @return EOK if no error, otherwise the type of error that occurred.
 */
errno_t ms5611_init_coefs(SensorLocation *loc, MS5611Context *ctx) {
    // PROM read command buffer
    i2c_sendrecv_t prom_read = {.stop = 1, .slave = loc->addr, .send_len = 1, .recv_len = 2};
    uint8_t prom_read_cmd[sizeof(prom_read) + sizeof(uint16_t)] = {0};
    memcpy(prom_read_cmd, &prom_read, sizeof(prom_read));

    // Read calibration data into sensor context
    errno_t err = devctl(loc->bus, DCMD_I2C_LOCK, NULL, 0, NULL); // Lock I2C bus
    if (err != EOK) return err;

    for (uint8_t i = 0; i < MS5611_COEFFICIENT_COUNT; i++) {

        // Read from PROM
        prom_read_cmd[sizeof(prom_read)] = CMD_PROM_RD + sizeof(uint16_t) * i; // Command to read next coefficient
        err = devctl(loc->bus, DCMD_I2C_SENDRECV, &prom_read_cmd, sizeof(prom_read_cmd), NULL);
        if (err != EOK) break;

        // Store calibration coefficient
        memcpy_be(&ctx->coefs[i], &prom_read_cmd[sizeof(prom_read)], sizeof(uint16_t));
    }

    err = devctl(loc->bus, DCMD_I2C_UNLOCK, NULL, 0, NULL); // Unlock I2C bus
    return err;
};
