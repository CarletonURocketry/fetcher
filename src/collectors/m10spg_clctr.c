#include "../drivers/m10spg/m10spg.h"
#include "../drivers/m10spg/ubx_def.h"
#include "../logging-utils/logging.h"
#include "collectors.h"

/**
 * Helper function to simplify sending a message on the message queue
 */
#define send_msg(sensor_q, msg, prio)                                                                                  \
    if (mq_send((sensor_q), (char *)(&(msg)), sizeof(msg), (prio)) == -1) {                                            \
        log_print(stderr, LOG_WARN, "M10SPG couldn't send message: %s.", strerror(errno));                             \
    }

void *m10spg_collector(void *args) {
    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        log_print(stderr, LOG_ERROR, "M10SPG collector could not open message queue '%s': '%s'", SENSOR_QUEUE,
                  strerror(errno));
        return (void *)((uint64_t)errno);
    }

    SensorLocation loc = {
        .bus = clctr_args(args)->bus,
        .addr = {.addr = (clctr_args(args)->addr), .fmt = I2C_ADDRFMT_7BIT},
    };

    int err;
    do {
        err = m10spg_open(&loc);
        if (err != EOK) {
            log_print(stderr, LOG_ERROR, "Could not open M10SPG: %s", strerror(err));
            return (void *)((uint64_t)err);
        }
    } while (err != EOK);

    for (;;) {
        UBXNavPVTPayload payload;
        common_t msg;
        err = m10spg_read(&loc, UBX_NAV_PVT, &payload, sizeof(payload));
        // Check if we could send command
        if (err == ENODATA) {
            // Nothing in the queue yet, but we expect there to be something soon, so keep reading
            continue;
        } else if (err) {
            log_print(stderr, LOG_ERROR, "Could not send command to M10SPG: %s", strerror(err));
            wait_for_meas(NULL);
            continue;
        }
        fetcher_log(stderr, LOG_INFO, "M10SPG current fix is: %d", payload.fixType);

        // Skip this payload if the fix isn't valid
        if (!(payload.flags & GNSS_FIX_OK)) {
            // Don't bother looking at data if it is going to be invalid
            log_print(stderr, LOG_WARN, "M10SPG fix is invalid or no-fix, skipping this payload ");
            wait_for_meas(&payload);
            continue;
        }

        // Only transmit data that is valid
        switch (payload.fixType) {
        case GPS_3D_FIX:
            msg.type = TAG_ALTITUDE_SEA;
            msg.data.FLOAT = (((float)payload.hMSL) / ALT_SCALE_TO_METERS);
            send_msg(sensor_q, msg, 2);
            // FALL THROUGH
        case GPS_FIX_DEAD_RECKONING:
            // FALL THROUGH
        case GPS_2D_FIX:
            // FALL THROUGH
        case GPS_DEAD_RECKONING:
            msg.type = TAG_COORDS;
            msg.data.VEC2D_I32.x = payload.lat;
            msg.data.VEC2D_I32.y = payload.lon;
            send_msg(sensor_q, msg, 3);
            break;
        case GPS_TIME_ONLY:
            break;
        case GPS_NO_FIX:
            break;
        default:
            break;
        }
        wait_for_meas(&payload);
    }
    log_print(stderr, LOG_ERROR, "M10SPG exited unexpectedly: %s", strerror(err));
    return (void *)((uint64_t)err);
}
