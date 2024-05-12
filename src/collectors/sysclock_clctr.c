#include "../drivers/sysclock/sysclock.h"
#include "collectors.h"
#include "sensor_api.h"
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
        return (void *)((uint64_t)err); // Extra uint64_t cast to silence compiler warning
    }

    // Data storage
    uint8_t data[sensor_max_dsize(&clock) + 1];
    printf("Size: %lu\n", sizeof(data));
    size_t nbytes;

    // Infinitely check the time
    for (;;) {

        clock.read(&clock, TAG_TIME, &data[1], &nbytes);
        data[0] = TAG_TIME; // Encode the contained data type

        // Infinitely send the time
        if (mq_send(sensor_q, (char *)data, sizeof(data), 0) == -1) {
            fprintf(stderr, "Sysclock failed to write message to sensor queue.\n");
        }
        usleep(10000); // Little sleep to not flood output
    }
}
