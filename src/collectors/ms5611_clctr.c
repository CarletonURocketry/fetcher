#include "../drivers/ms5611/ms5611.h"
#include "../drivers/sensor_api.h"
#include "collectors.h"
#include "logging.h"
#include <stdio.h>

#define return_errno(err) return (void *)((uint64_t)err)

/**
 * Collector thread for the MS5611 sensor.
 * @param args Arguments in the form of `collector_args_t`
 * @return The error `errno_t` which caused the thread to exit, encoded as a pointer.
 */
void *ms5611_collector(void *args) {

    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fetcher_log(stderr, LOG_ERROR, "MS5611 collector could not open message queue '%s': '%s'", SENSOR_QUEUE,
                    strerror(errno));
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
        fetcher_log(stderr, LOG_ERROR, "Failed to reset MS5611: %s\n", strerror(err));
        return_errno(err);
    }
    usleep(10000); // Takes some time to reset

    // Get the calibration coefficients
    MS5611Context ctx;
    err = ms5611_init_coefs(&loc, &ctx);

    if (err != EOK) {
        fetcher_log(stderr, LOG_ERROR, "Failed to initialize MS5611 calibration coefficients: %s", strerror(err));
        return_errno(err);
    }

    // Get the current pressure (ground pressure)
    err = ms5611_read_all(&loc, ADC_RES_4096, &ctx, 1, NULL, &ctx.ground_pressure, NULL);
    if (err != EOK) {
        fetcher_log(stderr, LOG_ERROR, "MS5611 failed to read ground pressure: %s", strerror(err));
        return_errno(err);
    }

    // Data storage
    common_t msg;
    double pressure;
    double altitude;
    double temperature;

    for (;;) {

        // Read all three data types
        err = ms5611_read_all(&loc, ADC_RES_4096, &ctx, 1, &temperature, &pressure, &altitude);

        // If read failed, just continue without crashing
        if (err != EOK) {
            fetcher_log(stderr, LOG_ERROR, "MS5611 failed to read data: %s", strerror(err));
            continue;
        }

        // Transmit temperature
        msg.type = TAG_TEMPERATURE;
        msg.data.FLOAT = (float)temperature;
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
            fetcher_log(stderr, LOG_ERROR, "MS5611 couldn't send message: %s.", strerror(errno));
        }

        // Transmit pressure
        msg.type = TAG_PRESSURE;
        msg.data.FLOAT = (float)pressure;
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 1) == -1) {
            fetcher_log(stderr, LOG_ERROR, "MS5611 couldn't send message: %s.", strerror(errno));
        }

        // Transmit altitude
        msg.type = TAG_ALTITUDE_REL;
        msg.data.FLOAT = (float)altitude;
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 2) == -1) {
            fetcher_log(stderr, LOG_ERROR, "MS5611 couldn't send message: %s.", strerror(errno));
        }
    }
}
