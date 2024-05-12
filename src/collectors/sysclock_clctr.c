#include "../drivers/sysclock/sysclock.h"
#include "collectors.h"

void *sysclock_collector(void *args) {

    // Create system clock instance
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
        sensor_write_data(stdout, TAG_TIME, &time);
        usleep(100); // Little sleep to not flood output
    }

    return (void *)EOK;
}
