/**
 * @file main.c
 * @brief The main function for the fetcher module, where program logic is used to create a console application.
 *
 * The main function for the fetcher module, where program logic is used to create a console application.
 */
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

/* Sensor includes */
#include "sensors/ms5611/ms5611.h"
#include "sensors/sensor_api.h"
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

    // Create MS5611 instance
    Sensor ms5611;
    ms5611_init(&ms5611, bus, 0x76, PRECISION_HIGH);

    uint8_t ms5611_context[sensor_get_ctx_size(ms5611)];
    sensor_set_ctx(ms5611, ms5611_context);
    errno_t setup_res = sensor_open(ms5611);
    if (setup_res != EOK) {
        fprintf(stderr, "%s\n", strerror(setup_res));
        exit(EXIT_FAILURE);
    }

    // Create system clock instance
    Sensor sysclock;
    sysclock_init(&sysclock, bus, 0x00, PRECISION_HIGH);
    uint8_t sysclock_context[sensor_get_ctx_size(sysclock)];
    sensor_set_ctx(sysclock, &sysclock_context);
    setup_res = sensor_open(sysclock);
    if (setup_res != EOK) {
        fprintf(stderr, "%s\n", strerror(setup_res));
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < 50; i++) {
        uint32_t data;
        uint8_t nbytes;
        sensor_read(sysclock, TAG_TIME, (uint8_t *)&data, &nbytes);
        sensor_print_data(TAG_TIME, &data);
        usleep(10000);
    }

    // Read temperature and pressure data
    while (!endless) {
        errno_t read_result;
        uint8_t nbytes;
        double data;
        for (uint8_t i = 0; i < ms5611.tag_list.len; i++) {
            SensorTag tag = ms5611.tag_list.tags[i];
            read_result = sensor_read(ms5611, tag, (uint8_t *)&data, &nbytes);
            if (read_result != EOK) {
                fprintf(stderr, "Could not read '%s' from MS5611: %s\n", sensor_strtag(tag), strerror(read_result));
            } else {
                sensor_print_data(tag, &data);
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
