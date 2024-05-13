#include <stdint.h>
#include <stdio.h>
#define SHT41_USE_CRC_LOOKUP
#include "../drivers/sht41/sht41.h"
#include "collectors.h"
#include "sensor_api.h"

/** Type to simplify sending measurements. */
struct sht41_msg_t {
    uint8_t type; /**< The measurement type (temperature, humidity). */
    float data;   /**< The measurement data. */
} __attribute__((packed));

/** Macro to cast `errno_t` to void pointer before returning. */
#define return_errno(err) return (void *)((uint64_t)err)

/**
 * Collector thread for the SHT41 sensor.
 * @param args Arguments in the form of `collector_args_t`
 * @return The error `errno_t` which caused the thread to exit, encoded as a pointer.
 */
void *sht41_collector(void *args) {

    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fprintf(stderr, "SHT41 collector could not open message queue '%s': '%s' \n", SENSOR_QUEUE, strerror(errno));
        return (void *)((uint64_t)errno);
    }

    /* Set up SHT41. */
    SensorLocation loc = {
        .bus = clctr_args(args)->bus,
        .addr = {.addr = clctr_args(args)->addr, .fmt = I2C_ADDRFMT_7BIT},
    };

    // Reset SHT41
    errno_t err = sht41_reset(&loc);
    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return_errno(err);
    }

    // Data storage
    float temperature;
    float humidity;
    struct sht41_msg_t msg;

    for (;;) {

        // Read temperature and humidity
        sht41_read(&loc, SHT41_HIGH_PRES, &temperature, &humidity);

        // Send temperature
        msg.type = TAG_TEMPERATURE;
        msg.data = temperature;
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
            fprintf(stderr, "SHT41 couldn't send message: %s\n", strerror(errno));
        }

        // Send humidity
        msg.type = TAG_HUMIDITY;
        msg.data = humidity;
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
            fprintf(stderr, "SHT41 couldn't send message: %s\n", strerror(errno));
        }
    }
}
