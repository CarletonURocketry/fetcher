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

    for (;;) {
        uint8_t id;
        pac195x_get_manu_id(&loc, &id);
        fprintf(stderr, "MANUID: %02x\n", id);
        pac195x_get_prod_id(&loc, &id);
        fprintf(stderr, "PRODID: %02x\n", id);
        pac195x_get_rev_id(&loc, &id);
        fprintf(stderr, "REVID: %02x\n", id);
    }

    return_err(EOK);
}
