#include "collectors.h"
#include "drivers/pac195x/pac195x.h"
#include <stdio.h>

/** Macro to early return errors. */
#define return_err(err) return (void *)((uint64_t)errno)

/** The RSENSE value connected to the PAC1952-2 in milliohms. */
#define RSENSE 18

typedef struct {
    uint8_t type;
    uint8_t id;
    int16_t voltage;
} voltage_msg_t;

void *pac1952_2_collector(void *args) {

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

    int err = pac195x_set_sample_mode(&loc, SAMPLE_1024_SPS_AD);
    if (err != EOK) {
        fprintf(stderr, "Failed to set sampling mode on PAC195X: %s\n", strerror(err));
        return_err(err);
    }

    err = pac195x_toggle_channel(&loc, CHANNEL1 | CHANNEL2, true);
    if (err != EOK) {
        fprintf(stderr, "Failed to enable all channels on PAC195X: %s\n", strerror(err));
        return_err(err);
    }

    err = pac195x_refresh(&loc); // Refresh after all configuration to force changes into effect
    usleep(1000);                // 1ms after refresh until accumulator data can be read again.
    if (err != EOK) {
        fprintf(stderr, "Failed to refresh PAC195X: %s\n", strerror(err));
        return_err(err);
    }

    uint16_t vbus[2];
    voltage_msg_t msg;

    for (;;) {

        for (int i = 0; i < 2; i++) {
            err = pac195x_get_vbusn(&loc, i + 1, &vbus[i]);
            if (err != EOK) {
                fprintf(stderr, "PAC195X could not read VBUS_%u: %s\n", i + 1, strerror(err));
                break;
            }
        }

        // Calculate voltage on SENSE 1
        msg.type = TAG_VOLTAGE;
        msg.id = 1;
        msg.voltage = pac195x_calc_bus_voltage(32, vbus[0], false);
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
            fprintf(stderr, "Could not send voltage measurement: %s\n", strerror(errno));
        }

        // Calculate voltage on SENSE 2
        msg.type = TAG_VOLTAGE;
        msg.id = 2;
        msg.voltage = pac195x_calc_bus_voltage(32, vbus[1], false);
        if (mq_send(sensor_q, (char *)&msg, sizeof(msg), 0) == -1) {
            fprintf(stderr, "Could not send voltage measurement: %s\n", strerror(errno));
        }

        // Get new measurements
        pac195x_refresh_v(&loc);
        usleep(1000);
    }

    return_err(EOK);
}
