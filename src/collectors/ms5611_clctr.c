#include "../drivers/ms5611/ms5611.h"
#include "collectors.h"
#include "sensor_api.h"
#include <stdio.h>

#define return_errno(err) return (void *)((uint64_t)err)

/** Type for easily sending measurements over the message queue. */
struct ms5611_message {
    uint8_t type; /**< Measurement type (temperature, pressure or altitude) */
    float data;   /**< Measurement data (temperature, pressure or altitude) */
} __attribute__((packed));

/**
 * Collector thread for the MS5611 sensor.
 * @param args Arguments in the form of `collector_args_t`
 * @return The error `errno_t` which caused the thread to exit, encoded as a pointer.
 */
void *ms5611_collector(void *args) {

    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fprintf(stderr, "MS5611 collector could not open message queue '%s': '%s' \n", SENSOR_QUEUE, strerror(errno));
        return (void *)((uint64_t)errno);
    }

    /* Configure MS5611. */
    SensorLocation loc = {
        .bus = clctr_args(args)->bus,
        .addr = {.addr = clctr_args(args)->addr, .fmt = I2C_ADDRFMT_7BIT},
    };

    // Reset the sensor
    errno_t err = ms5611_reset(&loc);
    if (err != EOK) {
        fprintf(stderr, "Failed to reset MS5611: %s\n", strerror(err));
        return_errno(err);
    }
    usleep(10000); // Takes some time to reset

    // Get the calibration coefficients
    MS5611Context ctx;
    err = ms5611_init_coefs(&loc, &ctx);

    if (err != EOK) {
        fprintf(stderr, "Failed to initialize MS5611 calibration coefficients: %s\n", strerror(err));
        return_errno(err);
    }

    // Get the current pressure (ground pressure)
    err = ms5611_read_all(&loc, ADC_RES_4096, &ctx, 1, NULL, &ctx.ground_pressure, NULL);
    if (err != EOK) {
        fprintf(stderr, "MS5611 failed to read ground pressure: %s\n", strerror(err));
        return_errno(err);
    }
    printf("GROUND PRESSURE: %f\n", ctx.ground_pressure);

    // Data storage
    struct ms5611_message measurement;
    double pressure;
    double altitude;
    double temperature;

    for (;;) {

        // Read all three data types
        err = ms5611_read_all(&loc, ADC_RES_4096, &ctx, 1, &temperature, &pressure, &altitude);

        // If read failed, just continue without crashing
        if (err != EOK) {
            fprintf(stderr, "MS5611 failed to read data: %s\n", strerror(err));
            continue;
        }

        // Transmit temperature
        measurement.type = TAG_TEMPERATURE;
        measurement.data = (float)temperature;
        if (mq_send(sensor_q, (char *)&measurement, sizeof(measurement), 0) == -1) {
            fprintf(stderr, "MS5611 couldn't send message: %s.\n", strerror(errno));
        }

        // Transmit pressure
        measurement.type = TAG_PRESSURE;
        measurement.data = (float)pressure;
        if (mq_send(sensor_q, (char *)&measurement, sizeof(measurement), 0) == -1) {
            fprintf(stderr, "MS5611 couldn't send message: %s.\n", strerror(errno));
        }

        // Transmit altitude
        measurement.type = TAG_ALTITUDE;
        measurement.data = (float)altitude;
        if (mq_send(sensor_q, (char *)&measurement, sizeof(measurement), 0) == -1) {
            fprintf(stderr, "MS5611 couldn't send message: %s.\n", strerror(errno));
        }
    }
}
