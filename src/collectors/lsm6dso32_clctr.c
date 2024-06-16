#include "../drivers/lsm6dso32/lsm6dso32.h"
#include "../drivers/sensor_api.h"
#include "../logging-utils/logging.h"
#include "collectors.h"
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
        log_print(stderr, LOG_ERROR, "LSM6DSO32 collector could not open message queue '%s': '%s'", SENSOR_QUEUE,
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
        log_print(stderr, LOG_ERROR, "Failed to reset LSM6DSO32: %s", strerror(err));
        return_err(err);
    }

    err = lsm6dso32_mem_reboot(&loc);
    if (err != EOK) {
        log_print(stderr, LOG_ERROR, "Failed to reboot LSM6DSO32 memory content: %s", strerror(err));
        return_err(err);
    }

    usleep(100);

    err = lsm6dso32_high_performance(&loc, true);
    if (err != EOK) {
        log_print(stderr, LOG_ERROR, "Failed to set LSM6DSO32 to high performance mode: %s", strerror(err));
        return_err(err);
    }

    err = lsm6dso32_set_acc_fsr(&loc, LA_FS_32G);
    if (err != EOK) {
        log_print(stderr, LOG_ERROR, "Failed to set LSM6DSO32 accelerometer FSR: %s", strerror(err));
        return_err(err);
    }

    err = lsm6dso32_set_gyro_fsr(&loc, G_FS_500);
    if (err != EOK) {
        log_print(stderr, LOG_ERROR, "Failed to set LSM6DSO32 gyroscope FSR: %s", strerror(err));
        return_err(err);
    }

    err = lsm6dso32_set_acc_odr(&loc, LA_ODR_6664);
    if (err != EOK) {
        log_print(stderr, LOG_ERROR, "Failed to set LSM6DSO32 accelerometer ODR: %s", strerror(err));
        return_err(err);
    }

    err = lsm6dso32_set_gyro_odr(&loc, G_ODR_6664);
    if (err != EOK) {
        log_print(stderr, LOG_ERROR, "Failed to set LSM6DSO32 gyroscope ODR: %s", strerror(err));
        return_err(err);
    }

    common_t msg;
    int16_t temperature;
    int16_t x;
    int16_t y;
    int16_t z;

    for (;;) {

        // Read temperature
        err = lsm6dso32_get_temp(&loc, &temperature);
        if (err != EOK) {
            log_print(stderr, LOG_ERROR, "LSM6DSO32 could not read temperature: %s", strerror(err));
        } else {
            msg.type = TAG_TEMPERATURE;
            msg.data.FLOAT = (float)temperature;
            if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
                log_print(stderr, LOG_ERROR, "LSM6DSO32 couldn't send message: %s", strerror(err));
            }
        }

        // Read linear acceleration
        err = lsm6dso32_get_accel(&loc, &x, &y, &z);
        if (err != EOK) {
            log_print(stderr, LOG_ERROR, "LSM6DSO32 could not read linear acceleration: %s", strerror(err));
        } else {
            lsm6dso32_convert_accel(LA_FS_32G, &x, &y, &z);
            msg.type = TAG_LINEAR_ACCEL_REL;
            msg.data.VEC3D = (vec3d_t){.x = x, .y = y, .z = z};
            if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 1) == -1) {
                log_print(stderr, LOG_ERROR, "LSM6DSO32 couldn't send message: %s", strerror(err));
            }
        }

        // Read angular velocity
        err = lsm6dso32_get_angular_vel(&loc, &x, &y, &z);

        if (err != EOK) {
            log_print(stderr, LOG_ERROR, "LSM6DSO32 could not read angular velocity: %s", strerror(err));
        } else {
            lsm6dso32_convert_angular_vel(G_FS_500, &x, &y, &z);
            msg.type = TAG_ANGULAR_VEL;
            msg.data.VEC3D = (vec3d_t){.x = x, .y = y, .z = z};
            if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
                log_print(stderr, LOG_ERROR, "LSM6DSO32 couldn't send message: %s", strerror(err));
            }
        }

        usleep(1000);
    }
}
