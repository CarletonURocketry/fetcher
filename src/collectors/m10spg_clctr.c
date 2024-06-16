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

int m10spg_pvt_handler(UBXFrame *msg);
// Share the sensor queue - not thread safe if different m10spg_collector threads use different message queues
mqd_t sensor_q;

void *m10spg_collector(void *args) {
    /* Open message queue. */
    sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        log_print(stderr, LOG_ERROR, "M10SPG collector could not open message queue '%s': '%s'", SENSOR_QUEUE,
                  strerror(errno));
        return (void *)((uint64_t)errno);
    }

    SensorLocation loc = {
        .bus = clctr_args(args)->bus,
        .addr = {.addr = (clctr_args(args)->addr), .fmt = I2C_ADDRFMT_7BIT},
    };
    M10SPGContext ctx;
    int err;
    do {
        err = m10spg_open(&ctx, &loc);
        if (err != EOK) {
            log_print(stderr, LOG_ERROR, "Could not open M10SPG: %s", strerror(err));
            continue;
        }
        err = m10spg_register_periodic(&ctx, m10spg_pvt_handler, UBX_MSG_NAV_PVT);
        if (err != EOK) {
            log_print(stderr, LOG_ERROR, "Could not configure periodic message: %s", strerror(err));
            continue;
        }
    } while (err != EOK);
    UBXFrame msg;
    UBXNavPVTPayload payload;
    msg.payload = &payload;
    for (;;) {
        // Clear out our read buffer (ask for nothing back)
        m10spg_read(&ctx, UBX_MSG_NONE, &msg, sizeof(payload));
        m10spg_sleep_epoch(&ctx);
    }
    log_print(stderr, LOG_ERROR, "M10SPG exited unexpectedly: %s", strerror(err));
    return (void *)((uint64_t)err);
}

/** A function implementing the M10SPGMessageHandler definition, for handling PVT messages (not thread safe currently)*/
int m10spg_pvt_handler(UBXFrame *msg) {
    if (!m10spg_is_type(msg, UBX_MSG_NAV_PVT)) {
        log_print(stderr, LOG_ERROR, "Handler was given a type it cannot handle, configuration error");
        return -1;
    }
    UBXNavPVTPayload *payload = msg->payload;
    log_print(stderr, LOG_INFO, "M10SPG current fix is: %d", payload->fixType);

    // Skip this payload if the fix isn't valid
    if (!(payload->flags & GNSS_FIX_OK)) {
        log_print(stderr, LOG_WARN, "M10SPG fix is invalid, skipping this payload");
        return 0;
    }

    // Only transmit data that is valid
    common_t to_send;
    switch (payload->fixType) {
    case GPS_3D_FIX:
        to_send.type = TAG_ALTITUDE_SEA;
        to_send.data.FLOAT = (((float)payload->hMSL) / ALT_SCALE_TO_METERS);
        send_msg(sensor_q, to_send, 2);
        // FALL THROUGH
    case GPS_FIX_DEAD_RECKONING:
        // FALL THROUGH
    case GPS_2D_FIX:
        // FALL THROUGH
    case GPS_DEAD_RECKONING:
        to_send.type = TAG_COORDS;
        to_send.data.VEC2D_I32.x = payload->lat;
        to_send.data.VEC2D_I32.y = payload->lon;
        send_msg(sensor_q, to_send, 3);
        break;
    case GPS_TIME_ONLY:
        break;
    case GPS_NO_FIX:
        break;
    default:
        break;
    }
    return 0;
}
