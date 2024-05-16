#include "../drivers/m10spg/m10spg.h"
#include "../drivers/m10spg/ubx_def.h"
#include "collectors.h"

union read_buffer {
    UBXNavPositionPayload pos;
    UBXNavVelocityPayload vel;
    UBXNavStatusPayload stat;
};

/** Type for sending measurements over the message queue. */
typedef struct {
    uint8_t type; /**< Measurement type */
    union {
        uint32_t U32;
    }; /**< Measurement data */
} message_t;

static void inline mq_send_msg(const mqd_t queue, message_t *msg) {
    if (mq_send(queue, (char *)&msg, sizeof(*msg), 0) == -1) {
        fprintf(stderr, "M10SPG couldn't send message: %s.\n", strerror(errno));
    }
}

void *m10spg_collector(void *args) {
    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fprintf(stderr, "M10SPG collector could not open message queue '%s': '%s' \n", SENSOR_QUEUE, strerror(errno));
        return (void *)((uint64_t)errno);
    }
    SensorLocation loc = {.bus = clctr_args(args)->bus,
                          .addr = {.addr = (clctr_args(args)->addr), .fmt = I2C_ADDRFMT_7BIT}};
    int err = m10spg_open(&loc);
    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return (void *)((uint64_t)err);
    }
    for (;;) {
        union read_buffer buf;
        message_t msg;
        err = m10spg_read(&loc, UBX_NAV_STAT, &buf, sizeof(UBXNavStatusPayload));
        // Check if we have a valid fix, no point logging bad data
        if (err == EOK) {
            // Check for the fix not being valid
            if (!((buf.stat.flags & 0x01) && buf.stat.gpsFix)) {
                // Instead of doing a continue, eventually should wait until the next nav epoch, or sleep off the i2c
                // buffer closing
                continue;
            }
        } else {
            fprintf(stderr, "M10SPG failed to read status: %s\n", strerror(err));
            continue;
        }
        err = m10spg_read(&loc, UBX_NAV_POSLLH, &buf, sizeof(UBXNavPositionPayload));
        if (err == EOK) {
            msg.type = TAG_LATITUDE;
            msg.U32 = buf.pos.lat;
            mq_send_msg(sensor_q, &msg);
            msg.type = TAG_LONGITUDE;
            msg.U32 = buf.pos.lon;
            mq_send_msg(sensor_q, &msg);
            msg.type = TAG_ALTITUDE;
            msg.U32 = buf.pos.hMSL;
            mq_send_msg(sensor_q, &msg);
        } else {
            fprintf(stderr, "M10SPG failed to read position: %s\n", strerror(err));
            continue;
        }
        // Read lat/long
        err = m10spg_read(&loc, UBX_NAV_VELNED, &buf, sizeof(UBXNavVelocityPayload));
        if (err == EOK) {
            msg.type = TAG_SPEED;
            msg.U32 = buf.vel.gSpeed;
            mq_send_msg(sensor_q, &msg);
            msg.type = TAG_COURSE;
            msg.U32 = buf.vel.heading;
            mq_send_msg(sensor_q, &msg);
        } else {
            fprintf(stderr, "M10SPG failed to read velocity: %s\n", strerror(err));
            continue;
        }
        // Read velocity
    }
    fprintf(stderr, "%s\n", strerror(err));
    return (void *)((uint64_t)err);
}