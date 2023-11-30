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
        fprintf(stderr, "Could not open I2C bus.\n");
        exit(EXIT_FAILURE);
    }

    // Create MS5611 instance
    Sensor ms5611;
    ms5611_init(&ms5611, bus, 0x76, PRECISION_HIGH);

    uint8_t ms5611_context[ms5611.context.size];
    ms5611.context.data = &ms5611_context;
    errno_t setup_res = ms5611.open(&ms5611);
    if (setup_res != EOK) {
        fprintf(stderr, "%s\n", strerror(setup_res));
        exit(EXIT_FAILURE);
    }

    // Create system clock instance
    Sensor sysclock;
    sysclock_init(&sysclock, bus, 0x00, PRECISION_HIGH);
    uint8_t sysclock_context[sysclock.context.size];
    sysclock.context.data = &sysclock_context;
    setup_res = sysclock.open(&sysclock);
    if (setup_res != EOK) {
        fprintf(stderr, "%s\n", strerror(setup_res));
        exit(EXIT_FAILURE);
    }

    for (int i = 0; i < 50; i++) {
        uint32_t data;
        uint8_t nbytes;
        sysclock.read(&sysclock, TAG_TIME, (uint8_t *)&data, &nbytes);
        printf("%s: %u %s\n", sensor_strtag(TAG_TIME), data, sensor_tag_unit(TAG_TIME));
        usleep(10000);
    }

    // Read temperature and pressure data
    while (!endless) {
        errno_t read_result;
        uint8_t nbytes;
        double data;
        for (uint8_t i = 0; i < ms5611.tag_list.len; i++) {
            SensorTag tag = ms5611.tag_list.tags[i];
            read_result = ms5611.read(&ms5611, tag, (uint8_t *)&data, &nbytes);
            if (read_result != EOK) {
                fprintf(stderr, "Could not read '%s' from MS5611: %s\n", sensor_strtag(tag), strerror(read_result));
            } else {
                printf("%s: %.2f %s\n", sensor_strtag(tag), data, sensor_tag_unit(tag));
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
