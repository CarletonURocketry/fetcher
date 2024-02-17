/**
 * @file main.c
 * @brief The main function for the fetcher module, where program logic is used to create a console application.
 *
 * The main function for the fetcher module, where program logic is used to create a console application.
 */
#include "sensors/sensor_api.h"
#include <devctl.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <hw/i2c.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Implemented sensors. */
#include "sensors/ms5611/ms5611.h"
#include "sensors/sysclock/sysclock.h"

/** Size of the buffer to read input data. */
#define BUFFER_SIZE 100

/* The speed of the I2C bus in kilobits per second. */
#define BUS_SPEED 400000

/** Flag to indicate reading from file in endless mode for debugging. */
static bool endless = false;

/** Name of file to read from, if one is provided. */
static char *filename = NULL;

int main(int argc, char **argv) {

    int c; // Holder for choice
    opterr = 0;

    /* Get command line options. */
    while ((c = getopt(argc, argv, ":e:")) != -1) {
        switch (c) {
        case 'e':
            endless = true;
            filename = optarg;
            break;
        case ':':
            fprintf(stderr, "Option -%c requires an argument.\n", optopt);
            exit(EXIT_FAILURE);
        case '?':
            fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            exit(EXIT_FAILURE);
        default:
            fputs("Something went wrong. Please check 'use fetcher' to see example usage.", stderr);
            exit(EXIT_FAILURE);
        }
    }

    /* Open I2C. */
    int bus = open("/dev/i2c1", O_RDWR);
    if (bus < 0) {
        fprintf(stderr, "Could not open I2C bus with error %s.\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* Set I2C bus speed. */
    uint32_t speed = BUS_SPEED;
    errno_t bus_speed = devctl(bus, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof(speed), NULL);
    if (bus_speed != EOK) {
        fprintf(stderr, "Failed to set bus speed to %u with error %s\n", speed, strerror(bus_speed));
        exit(EXIT_FAILURE);
    }

    /* Create sensor list. */
    Sensor sensors[2];

    // Create MS5611 instance
    ms5611_init(&sensors[0], bus, 0x77, PRECISION_HIGH);

    uint8_t ms5611_context[sensor_get_ctx_size(sensors[0])];
    sensor_set_ctx(&sensors[0], ms5611_context);
    errno_t setup_res = sensor_open(sensors[0]);
    if (setup_res != EOK) {
        fprintf(stderr, "%s\n", strerror(setup_res));
        exit(EXIT_FAILURE);
    }

    // Create system clock instance
    sysclock_init(&sensors[1], bus, 0x00, PRECISION_HIGH);

    uint8_t sysclock_context[sensor_get_ctx_size(sensors[1])];
    sensor_set_ctx(&sensors[1], sysclock_context);
    setup_res = sensor_open(sensors[1]);
    if (setup_res != EOK) {
        fprintf(stderr, "%s\n", strerror(setup_res));
        exit(EXIT_FAILURE);
    }

    // Read all sensor data
    while (!endless) {
        errno_t read_result; // The result of a sensor read
        size_t nbytes;       // The number of bytes returned by a sensor read
        for (uint8_t i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
            Sensor sensor = sensors[i];              // Grab the current sensor
            uint8_t data[sensor_max_dsize(&sensor)]; // Allocate sufficient data to read the sensor

            /* Read all of the possible data the sensor can provide, and print it to stdout. */
            for (uint8_t j = 0; j < sensor.tag_list.len; j++) {
                SensorTag tag = sensor.tag_list.tags[j];
                read_result = sensor_read(sensor, tag, data, &nbytes);

                // Sensor read didn't work, skip this iteration
                if (read_result != EOK) {
                    fprintf(stderr, "Could not read sensor data: %s\n", sensor_strtag(tag));
                    continue;
                }
                sensor_print_data(tag, data); // Sensor read worked, print out the result
            }
        }
    }

    // Only read from a file if in endless mode
    if (endless) {

        /* Open file for reading. */
        FILE *f = fopen(filename, "r");
        if (f == NULL) {
            fprintf(stderr, "File '%s' cannot be opened.\n", filename);
            exit(EXIT_FAILURE);
        }

        /* Loop over file to get data. */
        uint8_t buffer[BUFFER_SIZE] = {0};
        for (;;) {
            size_t items_read = fread(&buffer, sizeof(uint8_t), BUFFER_SIZE, f);
            fwrite(&buffer, sizeof(uint8_t), items_read, stdout);

            // In endless mode, go back to the file start when we reach the end of the file
            if (feof(f)) {
                if (endless) {
                    rewind(f);
                } else {
                    return 0; // Otherwise complete
                }
            }
        }
    }

    return 0;
}
