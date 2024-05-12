#include "../drivers/ms5611/ms5611.h"
#include "collectors.h"
#include "sensor_api.h"

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

    /* Create MS5611 instance */
    Sensor ms5611;
    ms5611_init(&ms5611, clctr_args(args)->bus, clctr_args(args)->addr, PRECISION_HIGH);
    uint8_t ms5611_context[sensor_get_ctx_size(ms5611)];
    sensor_set_ctx(&ms5611, ms5611_context);
    errno_t err = sensor_open(ms5611);
    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return (void *)((uint64_t)err);
    }

    uint8_t data[sensor_max_dsize(&ms5611) + 1];
    size_t nbytes;

    for (;;) {

        // Read pressure
        sensor_read(ms5611, TAG_PRESSURE, &data[1], &nbytes);
        data[0] = TAG_PRESSURE;
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "MS5611 couldn't send message: %s.\n", strerror(errno));
        }

        // Read temperature
        sensor_read(ms5611, TAG_TEMPERATURE, &data[1], &nbytes);
        data[0] = TAG_TEMPERATURE;
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "MS5611 couldn't send message: %s.\n", strerror(errno));
        }
    }
}
