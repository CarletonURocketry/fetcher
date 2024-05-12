#include "../drivers/lsm6dso32/lsm6dso32.h"
#include "collectors.h"
#include "sensor_api.h"
#include <stdio.h>

void *lsm6dso32_collector(void *args) {

    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fprintf(stderr, "LSM6DSO32 collector could not open message queue '%s': '%s' \n", SENSOR_QUEUE,
                strerror(errno));
        return (void *)((uint64_t)errno);
    }

    /* Create LSM6DO32 instance */
    Sensor lsm6dso32;
    lsm6dso32_init(&lsm6dso32, clctr_args(args)->bus, clctr_args(args)->addr, PRECISION_HIGH);

    uint8_t lsm6dso32_context[sensor_get_ctx_size(lsm6dso32)];
    sensor_set_ctx(&lsm6dso32, lsm6dso32_context);
    errno_t err = sensor_open(lsm6dso32);
    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return (void *)((uint64_t)err);
    }

    size_t nbytes;
    uint8_t data[sensor_max_dsize(&lsm6dso32) + 1];

    for (;;) {

        // Read temperature
        sensor_read(lsm6dso32, TAG_TEMPERATURE, &data[1], &nbytes);
        data[0] = TAG_TEMPERATURE;
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "LSM6DSO32 couldn't send message: %s\n", strerror(errno));
        }

        // Read linear acceleration
        sensor_read(lsm6dso32, TAG_LINEAR_ACCEL, &data[1], &nbytes);
        data[0] = TAG_LINEAR_ACCEL;
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "LSM6DSO32 couldn't send message: %s\n", strerror(errno));
        }

        // Read angular velocity
        sensor_read(lsm6dso32, TAG_ANGULAR_VEL, &data[1], &nbytes);
        data[0] = TAG_ANGULAR_VEL;
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "LSM6DSO32 couldn't send message: %s\n", strerror(errno));
        }
    }
}
