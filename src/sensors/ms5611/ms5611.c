/**
 * @file ms5611.c
 * @brief Sensor API interface implementation for the MS5611 pressure and temperature sensor.
 *
 * Sensor API interface implementation for the MS5611 pressure and temperature sensor which uses I2C communication.
 */
#include "ms5611.h"
#include "../sensor_api.h"
#include <errno.h>
#include <hw/i2c.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

/** Number of calibration coefficients */
#define NUM_COEFFICIENTS 8
/** Size of coefficients in bytes */
#define COEF_TYPE uint16_t

/** Macro to early return an error. */
#define return_err(err)                                                                                                \
    if (err != EOK) return err

/** A list of data types that can be read by the MS5611 */
static const SensorTag TAGS[] = {TAG_TEMPERATURE, TAG_PRESSURE};

/** All of the I2C commands that can be used on the MS5611 sensor. */
typedef enum ms5611_cmd_t {
    CMD_RESET = 0x1E,    /**< ADC reset command */
    CMD_PROM_RD = 0xA0,  /**< Prom read command */
    CMD_ADC_CONV = 0x40, /**< ADC conversion command */
    CMD_ADC_READ = 0x00, /**< ADC read command */
} MS5611Cmd;

/** The D registers on the MS5611 sensor. */
typedef enum ms5611_dregs_t {
    D1 = 0x00, /**< D register 1 */
    D2 = 0x10, /**< D register 2 */
} MS5611DReg;

/** The resolution that is used to read the data of the MS5611 */
typedef enum ms5611_resolution_t {
    ADC_RES_256 = 0x00,  /**< ADC OSR=256 */
    ADC_RES_512 = 0x02,  /**< ADC OSR=512 */
    ADC_RES_1024 = 0x04, /**< ADC OSR=1024 */
    ADC_RES_2048 = 0x06, /**< ADC OSR=2048 */
    ADC_RES_4096 = 0x08, /**< ADC OSR=4096 */
} MS5611Resolution;

/** Implementation of precision in sensor API for read resolution. */
static const MS5611Resolution PRECISIONS[] = {
    [PRECISION_LOW] = ADC_RES_256,
    [PRECISION_MED] = ADC_RES_1024,
    [PRECISION_HIGH] = ADC_RES_4096,
};

/**
 * Resets the MS5611 sensor, ensuring that calibration data is loaded into the PROM.
 * @param loc The sensor's location on the I2C bus.
 * @return Any error from the attempted reset. EOK if successful.
 */
static errno_t ms5611_reset(SensorLocation *loc) {
    i2c_send_t reset = {.stop = 1, .len = 1, .slave = loc->addr};
    uint8_t reset_cmd[sizeof(i2c_send_t) + 1];
    memcpy(reset_cmd, &reset, sizeof(reset));
    reset_cmd[sizeof(reset)] = CMD_RESET;
    return devctl(loc->bus, DCMD_I2C_SEND, &reset_cmd, sizeof(reset_cmd), NULL);
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
    errno_t result = devctl(loc->bus, DCMD_I2C_SEND, &conversion_cmd, sizeof(conversion_cmd), NULL);
    return_err(result);

    // Wait for appropriate conversion time
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

    // Read D register
    i2c_sendrecv_t read = {.slave = loc->addr, .stop = 1, .send_len = 1, .recv_len = 3};
    uint8_t read_cmd[sizeof(read) + 3];
    memcpy(read_cmd, &read, sizeof(read));
    read_cmd[sizeof(read)] = CMD_ADC_READ;
    result = devctl(loc->bus, DCMD_I2C_SENDRECV, &read_cmd, sizeof(read_cmd), NULL);
    return_err(result);

    *value = 0;
    *value += read_cmd[sizeof(read)] * 65536;
    *value += read_cmd[sizeof(read) + 1] * 256;
    *value += read_cmd[sizeof(read) + 2];

    return EOK;
}

/**
 * Prepares the MS5611 for reading and initializes the sensor context with the calibration coefficients.
 * @param sensor A reference to an MS5611 sensor.
 * @return The error status of the call. EOK if successful.
 */
static errno_t ms5611_open(Sensor *sensor) {

    // Load calibration into PROM
    errno_t op_status = ms5611_reset(&sensor->loc);
    return_err(op_status);
    usleep(10000); // Takes some time to reset

    // PROM read command buffer
    i2c_sendrecv_t prom_read = {.stop = 1, .slave = sensor->loc.addr, .send_len = 1, .recv_len = 2};
    uint8_t prom_read_cmd[sizeof(prom_read) + sizeof(COEF_TYPE)] = {0};
    memcpy(prom_read_cmd, &prom_read, sizeof(prom_read));

    // Read calibration data into sensor context
    COEF_TYPE *cal_coefs = (COEF_TYPE *)sensor->context.data;
    for (uint8_t i = 0; i < NUM_COEFFICIENTS; i++) {

        // Read from PROM
        prom_read_cmd[sizeof(prom_read)] = CMD_PROM_RD + sizeof(COEF_TYPE) * i; // Command to read next coefficient
        op_status = devctl(sensor->loc.bus, DCMD_I2C_SENDRECV, &prom_read_cmd, sizeof(prom_read_cmd), NULL);
        return_err(op_status);

        // Store calibration coefficient
        memcpy_be(&cal_coefs[i], &prom_read_cmd[sizeof(prom_read)], sizeof(COEF_TYPE));
    }
    return EOK;
}

/**
 * Reads the specified data from the MS5611.
 * @param sensor A reference to an MS5611 sensor.
 * @param tag The tag of the data type that should be read.
 * @param buf A pointer to the byte array to store the data.
 * @param nbytes The number of bytes that were written into the byte array buffer.
 * @return Error status of reading from the sensor. EOK if successful.
 */
static errno_t ms5611_read(Sensor *sensor, const SensorTag tag, uint8_t *buf, uint8_t *nbytes) {

    // Read D registers with configured precision
    uint32_t d1, d2;
    errno_t dread_res = ms5611_read_dreg(&sensor->loc, D1 + PRECISIONS[sensor->precision], &d1);
    return_err(dread_res);
    dread_res = ms5611_read_dreg(&sensor->loc, D2 + PRECISIONS[sensor->precision], &d2);
    return_err(dread_res);

    // Calculate 1st order pressure and temperature (MS5607 1st order algorithm)
    COEF_TYPE *coefs = (COEF_TYPE *)sensor->context.data;

    double dt = d2 - coefs[5] * pow(2, 8);
    double off = coefs[2] * pow(2, 16) + dt * coefs[4] / pow(2, 7);
    double sens = coefs[1] * pow(2, 15) + dt * coefs[3] / pow(2, 8);

    double temperature = (2000 + ((dt * coefs[6]) / pow(2, 23)));

    // Second order algorithm (only if high precision was chosen)
    if (sensor->precision == PRECISION_HIGH) {
        double t2 = 0;
        double off2 = 0;
        double sens2 = 0;

        if (temperature < 20) {
            t2 = (dt * dt) / pow(2, 31);
            double temp_square = pow((temperature - 2000), 2);
            off2 = (5 * temp_square) / 2;
            sens2 = (5 * temp_square) / 4;

            if (temperature < -15) {
                temp_square = pow((temperature + 1500), 2);
                off2 = off2 + 7 * temp_square;
                sens2 = sens2 + 11 * temp_square / 2;
            }
        }

        temperature -= t2;
        off -= off2;
        sens -= sens2;
    }

    switch (tag) {
    case TAG_TEMPERATURE: {
        temperature /= 100; // Degrees C
        memcpy(buf, &temperature, sizeof(temperature));
        *nbytes = sizeof(temperature);
    } break;
    case TAG_PRESSURE: {
        double pressure = (((d1 * sens) / (pow(2, 21)) - off) / pow(2, 15)) / 1000; // kPa
        memcpy(buf, &pressure, sizeof(pressure));
        *nbytes = sizeof(pressure);
        break;
    }
    default:
        return EINVAL;
    }

    return EOK;
}

/**
 * Initializes a sensor struct with the interface to interact with the MS5611.
 * @param sensor The sensor interface to be initialized.
 * @param bus The file descriptor of the I2C bus.
 * @param addr The address of the sensor on the I2C bus.
 * @param precision The precision to read measurements with.
 */
void ms5611_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision) {
    sensor->precision = precision;
    sensor->loc = (SensorLocation){.bus = bus, .addr = {.addr = (addr & 0x7F), .fmt = I2C_ADDRFMT_7BIT}};
    sensor->max_return_size = sizeof(double);
    sensor->tag_list = (SensorTagList){.tags = TAGS, .tag_count = sizeof(TAGS) / sizeof(SensorTag)};
    sensor->context.size = (NUM_COEFFICIENTS * sizeof(COEF_TYPE)); // Size of all calibration data
    sensor->open = &ms5611_open;
    sensor->read = &ms5611_read;
}
