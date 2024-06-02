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
        uint32_t U32;  /**< The message payload, interpreted as a uin32_t */
        int32_t I32;   /**< The message payload, interpreted as a int32_t */
        uint8_t U8;    /**< The message payload, interpreted as a uint8_t */
        vec2d_t VEC2D; /**< The message payload, interpreted as a vec2d_t */
    };
} __attribute__((packed)) message_t;

void *m10spg_collector(void *args) {

    /* Open message queue. */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_WRONLY);
    if (sensor_q == -1) {
        fprintf(stderr, "M10SPG collector could not open message queue '%s': '%s' \n", SENSOR_QUEUE, strerror(errno));
        return (void *)((uint64_t)errno);
    }

    SensorLocation loc = {
        .bus = clctr_args(args)->bus,
        .addr = {.addr = (clctr_args(args)->addr), .fmt = I2C_ADDRFMT_7BIT},
    };

    int err = m10spg_open(&loc);
    if (err != EOK) {
        fprintf(stderr, "Could not open M10SPG: %s\n", strerror(err));
        return (void *)((uint64_t)err);
    }

    for (;;) {
        // TODO - Don't read if the next epoch hasn't happened
        union read_buffer buf;
        message_t msg;
        /* err = m10spg_send_command(&loc, UBX_NAV_STAT, &buf, sizeof(UBXNavStatusPayload)); */
        /* // Check if we have a valid fix, no point logging bad data */
        /* if (err == EOK) { */
        /*     // Make sure that the fix is valid (has a reasonable value and is not a no-fix) */
        /*     if ((buf.stat.flags & 0x01) && buf.stat.gpsFix) { */
        /*         msg.type = TAG_FIX; */
        /*         msg.U8 = buf.stat.gpsFix; */
        /*         if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) { */
        /*             fprintf(stderr, "M10SPG couldn't send message: %s.\n", strerror(errno)); */
        /*         } */
        // The else here is commented out so you can see the data processing is working even while an invalid
        // fix is held
        /*     } // else { */
        /*     //    // Instead of doing a continue, should sleep until the next epoch */
        /*     //    printf("Bad GPS fix, skipping\n"); */
        /*     //    continue; */
        /*     //} */
        /* } else { */
        /*     fprintf(stderr, "M10SPG failed to read status: %s\n", strerror(err)); */
        /*     continue; */
        /* } */

        // Read position
        err = m10spg_send_command(&loc, UBX_NAV_POSLLH, &buf, sizeof(UBXNavPositionPayload));
        if (err == EOK) {
            msg.type = TAG_COORDS;
            msg.VEC2D.x = ((float)buf.pos.lat / LAT_SCALE_TO_DEGREES);
            msg.VEC2D.y = ((float)buf.pos.lon / LON_SCALE_TO_DEGREES);

            if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
                fprintf(stderr, "M10SPG couldn't send message: %s.\n", strerror(errno));
            }
            msg.type = TAG_ALTITUDE_SEA;
            msg.I32 = buf.pos.hMSL;
            if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
                fprintf(stderr, "M10SPG couldn't send message: %s.\n", strerror(errno));
            }
        } else {
            fprintf(stderr, "M10SPG failed to read position: %s\n", strerror(err));
            continue;
        }

        // Read velocity
        /* err = m10spg_send_command(&loc, UBX_NAV_VELNED, &buf, sizeof(UBXNavVelocityPayload)); */
        /* if (err == EOK) { */
        /*     msg.type = TAG_SPEED; */
        /*     msg.U32 = buf.vel.gSpeed; */
        /*     if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) { */
        /*         fprintf(stderr, "M10SPG couldn't send message: %s.\n", strerror(errno)); */
        /*     } */
        /*     msg.type = TAG_COURSE; */
        /*     msg.U32 = buf.vel.heading; */
        /*     if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) { */
        /*         fprintf(stderr, "M10SPG couldn't send message: %s.\n", strerror(errno)); */
        /*     } */
        /* } else { */
        /*     fprintf(stderr, "M10SPG failed to read velocity: %s\n", strerror(err)); */
        /*     continue; */
        /* } */
    }
    fprintf(stderr, "%s\n", strerror(err));
    return (void *)((uint64_t)err);
}
