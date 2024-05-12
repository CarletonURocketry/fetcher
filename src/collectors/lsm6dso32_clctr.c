#include "../drivers/lsm6dso32/lsm6dso32.h"
#include "collectors.h"
#include "sensor_api.h"

void *lsm6dso32_collector(void *args) {

    // Create LSM6DO32 instance
    Sensor lsm6dso32;
    lsm6dso32_init(&lsm6dso32, clctr_args(args)->bus, clctr_args(args)->addr, PRECISION_HIGH);

    uint8_t lsm6dso32_context[sensor_get_ctx_size(lsm6dso32)];
    sensor_set_ctx(&lsm6dso32, lsm6dso32_context);
    errno_t err = sensor_open(lsm6dso32);
    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return (void *)err;
    }

    size_t nbytes;
    float temperature;
    vec3d_t vector_data;
    for (;;) {
        // Read temperature
        sensor_read(lsm6dso32, TAG_TEMPERATURE, &temperature, &nbytes);
        sensor_write_data(stdout, TAG_TEMPERATURE, &temperature);
        // Read linear acceleration
        sensor_read(lsm6dso32, TAG_LINEAR_ACCEL, &vector_data, &nbytes);
        sensor_write_data(stdout, TAG_LINEAR_ACCEL, &vector_data);
        // Read angular velocity
        sensor_read(lsm6dso32, TAG_ANGULAR_VEL, &vector_data, &nbytes);
        sensor_write_data(stdout, TAG_ANGULAR_VEL, &vector_data);
    }
}
