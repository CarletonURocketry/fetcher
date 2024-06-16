#include "../drivers/sensor_api.h"
#include "../logging-utils/logging.h"
#include "collectors.h"
#include <errno.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

/** Macro to cast `errno_t` to void pointer before returning. */
#define return_err(err) return (void *)((uint64_t)err)

/**
 * Collector thread for the system clock.
 * @param args Arguments in the form of `collector_args_t`
 * @return The error `errno_t` which caused the thread to exit, encoded as a pointer.
 */
void *sysclock_collector(void *args) {

    (void)(args); // Ignore that args is unused

    /* Open message queue to send data. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        log_print(stderr, LOG_ERROR, "Sysclock collector could not open message queue '%s': '%s'", SENSOR_QUEUE,
                  strerror(errno));
        return (void *)((uint64_t)errno);
    }

    // Get the current UNIX time and time information
    struct timespec start;
    int err = clock_gettime(CLOCK_REALTIME, &start);
    if (err) {
        log_print(stderr, LOG_ERROR, "Could not get startup time: %s", strerror(errno));
        return_err(errno);
    }

    // Infinitely check the time
    struct timespec now;
    common_t msg;
    msg.type = TAG_TIME;
    for (;;) {

        // Get time with nanosecond precision
        err = clock_gettime(CLOCK_REALTIME, &now);
        if (err) {
            log_print(stderr, LOG_ERROR, "Could not get current time: %s", strerror(errno));
            continue;
        }

        // Calculate elapsed time from launch
        long elapsed_s = now.tv_sec - start.tv_sec;
        long elapsed_ns = now.tv_nsec - start.tv_nsec;
        msg.data.U32 = (elapsed_s * 1000) + (elapsed_ns / 1000000);

        // Infinitely send the time
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
            log_print(stderr, LOG_ERROR, "Sysclock couldn't send message: %s.", strerror(errno));
        }
        usleep(10000); // Little sleep to not flood message queue
    }
}
