#include <stdint.h>
#include <stdio.h>
#define SHT41_USE_CRC_LOOKUP
#include "../drivers/sht41/sht41.h"
#include "collectors.h"
#include "sensor_api.h"

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

    /* Create SHT41 instance. */
    Sensor sht41;
    sht41_init(&sht41, clctr_args(args)->bus, clctr_args(args)->addr, PRECISION_HIGH);

    uint8_t sht41_context[sensor_get_ctx_size(sht41)];

    sensor_set_ctx(&sht41, sht41_context);
    errno_t err = sensor_open(sht41);
    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return (void *)((uint64_t)err); // Extra uint64_t cast to silence compiler warning
    }

    size_t nbytes;
    uint8_t data[sensor_max_dsize(&sht41) + 1];

    for (;;) {

        // Read temperature
        sensor_read(sht41, TAG_TEMPERATURE, &data[1], &nbytes);
        data[0] = TAG_TEMPERATURE;
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "SHT41 couldn't send message: %s\n", strerror(errno));
        }

        // Read humidity
        sensor_read(sht41, TAG_HUMIDITY, &data[1], &nbytes);
        data[0] = TAG_HUMIDITY;
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "SHT41 couldn't send message: %s\n", strerror(errno));
        }
    }
}
