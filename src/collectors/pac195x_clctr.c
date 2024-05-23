#include "collectors.h"
#include "drivers/pac195x/pac195x.h"
#include <stdio.h>

#define return_err(err) return (void *)((uint64_t)errno)

void *pac195x_collector(void *args) {

    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fprintf(stderr, "PAC195X collector could not open message queue '%s': '%s' \n", SENSOR_QUEUE, strerror(errno));
        return_err(err);
    }

    SensorLocation loc = {
        .addr = {.addr = clctr_args(args)->addr, .fmt = I2C_ADDRFMT_7BIT},
        .bus = clctr_args(args)->bus,
    };

    int err = pac195x_refresh(&loc);
    if (err != EOK) {
        fprintf(stderr, "Failed to refresh PAC195X: %s\n", strerror(err));
        return_err(err);
    }

    for (;;) {
    }

    return_err(EOK);
}
