#include "../drivers/lsm6dso32/lsm6dso32.h"
#include "collectors.h"
#include "sensor_api.h"
#include <stdio.h>

#define return_err(err) return (void *)((uint64_t)errno)

/**
 * Collector thread for the LSM6DSO32 sensor.
 * @param args Arguments in the form of `collector_args_t`
 * @return The error `errno_t` which caused the thread to exit, encoded as a pointer.
 */
void *lsm6dso32_collector(void *args) {

    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fprintf(stderr, "LSM6DSO32 collector could not open message queue '%s': '%s' \n", SENSOR_QUEUE,
                strerror(errno));
        return_err(err);
    }

    SensorLocation loc = {
        .addr = {.addr = clctr_args(args)->addr, .fmt = I2C_ADDRFMT_7BIT},
        .bus = clctr_args(args)->bus,
    };

    int err;
    err = lsm6dso32_reset(&loc);
    if (err != EOK) {
        fprintf(stderr, "Failed to reset LSM6DSO32: %s\n", strerror(errno));
        return_err(err);
    }

    err = lsm6dso32_set_acc_fsr(&loc, LA_FS_32G);
    if (err != EOK) {
        fprintf(stderr, "Failed to set LSM6DSO32 accelerometer FSR: %s\n", strerror(errno));
        return_err(err);
    }

    err = lsm6dso32_set_gyro_fsr(&loc, G_FS_500);
    if (err != EOK) {
        fprintf(stderr, "Failed to set LSM6DSO32 gyroscope FSR: %s\n", strerror(errno));
        return_err(err);
    }

    uint8_t data[sizeof(vec3d_t) + 1];
    int16_t temperature;
    int16_t x;
    int16_t y;
    int16_t z;

    for (;;) {

        // Read temperature
        lsm6dso32_get_temp(&loc, &temperature);
        data[0] = TAG_TEMPERATURE;
        *((float *)(data + 1)) = (float)temperature;
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "LSM6DSO32 couldn't send message: %s\n", strerror(errno));
        }

        // Read linear acceleration
        lsm6dso32_get_accel(&loc, &x, &y, &z);
        data[0] = TAG_LINEAR_ACCEL_REL;
        *((vec3d_t *)(data + 1)) = (vec3d_t){.x = x, .y = y, .z = z};
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "LSM6DSO32 couldn't send message: %s\n", strerror(errno));
        }

        // Read angular velocity
        lsm6dso32_get_angular_vel(&loc, &x, &y, &z);
        data[0] = TAG_ANGULAR_VEL;
        *((vec3d_t *)(data + 1)) = (vec3d_t){.x = x, .y = y, .z = z};
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "LSM6DSO32 couldn't send message: %s\n", strerror(errno));
        }
    }
}
