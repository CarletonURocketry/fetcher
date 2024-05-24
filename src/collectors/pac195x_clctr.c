#include "collectors.h"
#include "drivers/pac195x/pac195x.h"
#include <stdio.h>

#define return_err(err) return (void *)((uint64_t)errno)

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

    for (;;) {
        uint16_t vsense[2];
        uint16_t vbus[2];
        uint32_t vpower[2];

        for (int i = 0; i < 2; i++) {
            err = pac195x_get_vsensen(&loc, i + 1, &vsense[i]);
            if (err != EOK) {
                fprintf(stderr, "PAC195X could not read VSENSE%d: %s\n", i - 1, strerror(err));
                break;
            }
        }

        for (int i = 0; i < 2; i++) {
            err = pac195x_get_vbusn(&loc, i + 1, &vbus[i]);
            if (err != EOK) {
                fprintf(stderr, "PAC195X could not read VBUS_%d: %s\n", i - 1, strerror(err));
                break;
            }
        }

        // Calculate voltage on SENSE
        printf("SENSE1+ VOLTAGE: %umV\n", pac195x_calc_bus_voltage(32, vbus[0], false));
        printf("SENSE2+ VOLTAGE: %umV\n", pac195x_calc_bus_voltage(32, vbus[1], false));

        // Calculate current on SENSE
        printf("VSENSE1: %u\n", vsense[0]);
        printf("VSENSE2: %u\n", vsense[1]);

        printf("SENSE1+ CURRENT: %uuA\n", pac195x_calc_bus_current(1, vsense[0], false));
        printf("SENSE2+ CURRENT: %uuA\n", pac195x_calc_bus_current(1, vsense[1], false));

        for (int i = 0; i < 2; i++) {
            err = pac195x_get_powern(&loc, i + 1, &vpower[i]);
            if (err != EOK) {
                fprintf(stderr, "PAC195X could not read POWER_%d: %s\n", i - 1, strerror(err));
                break;
            }
        }

        pac195x_refresh_v(&loc);
        usleep(1000);
    }

    return_err(EOK);
}
