#include "../drivers/sysclock/sysclock.h"
#include "collectors.h"
#include <stdio.h>
#include <time.h>

void *sysclock_collector(void *args) {

    /* Open message queue to send data. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);

    /* Create system clock instance. */
    Sensor clock;
    sysclock_init(&clock, clctr_args(args)->bus, clctr_args(args)->addr, PRECISION_HIGH);
    uint8_t sysclock_context[sensor_get_ctx_size(clock)];
    sensor_set_ctx(&clock, sysclock_context);

    errno_t err = sensor_open(clock);

    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return (void *)err;
    }

    uint32_t time;
    size_t nbytes;
    for (;;) {

        // Infinitely check the time
        clock.read(&clock, TAG_TIME, &time, &nbytes);

        // Infinitely send the time
        char time_str[30];
        sprintf(time_str, "Time: %u\n", time);
        mq_send(sensor_q, time_str, sizeof(time_str), 0);
        usleep(10000); // Little sleep to not flood output
    }

    return (void *)EOK;
}
