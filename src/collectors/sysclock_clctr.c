#include "collectors.h"
#include "sensor_api.h"
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

/** Type to simplify sending time data. */
struct sysclock_msg_t {
    uint8_t type;    /**< The type of message (always time). */
    uint32_t millis; /**< The number of milliseconds since launch. */
} __attribute__((packed));

/**
 * Collector thread for the system clock.
 * @param args Arguments in the form of `collector_args_t`
 * @return The error `errno_t` which caused the thread to exit, encoded as a pointer.
 */
void *sysclock_collector(void *args) {

    /* Open message queue to send data. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fprintf(stderr, "Sysclock collector could not open message queue '%s': '%s' \n", SENSOR_QUEUE, strerror(errno));
        return (void *)((uint64_t)errno);
    }

    // Get the current UNIX time and time information
    time_t start_unix_time;
    time(&start_unix_time);
    struct tm *time_info = localtime(&start_unix_time);
    struct timezone tz = {.tz_dsttime = 0, .tz_minuteswest = time_info->tm_gmtoff};
    struct timeval tval;

    // Infinitely check the time
    struct sysclock_msg_t msg;
    msg.type = TAG_TIME;
    for (;;) {

        // Get time with nanosecond precision
        gettimeofday(&tval, &tz);

        // Calculate elapsed time from launch
        time_t elapsed_s = tval.tv_sec - start_unix_time;
        msg.millis = (elapsed_s * 1000) + (tval.tv_usec / 1000);

        // Infinitely send the time
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
            fprintf(stderr, "Sysclock couldn't send message: %s.\n", strerror(errno));
        }
        usleep(10000); // Little sleep to not flood message queue
    }
}
