/**
 * @file main.c
 * @brief The main function for the fetcher module, where program logic is used to create a console application.
 *
 * The main function for the fetcher module, where program logic is used to create a console application.
 */
#include "collectors/collectors.h"
#include "eeprom/eeprom.h"
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
#include <time.h>

/** Size of the buffer to read input data. */
#define BUFFER_SIZE 100

/* The speed of the I2C bus in kilobits per second. */
#define BUS_SPEED 400000

/** The size of the static memory buffer (in bytes) for allocating sensor contexts on. */
#define ARENA_SIZE 256

/** Flag to indicate reading from file in endless mode for debugging. */
static bool endless = false;

/** Name of file to read from, if one is provided. */
static char *filename = NULL;

/** Name of file to write to, if one is provided. */
static char *outfile = NULL;

int main(int argc, char **argv) {

    int c; // Holder for choice
    opterr = 0;

    /* Get command line options. */
    while ((c = getopt(argc, argv, ":e:o:")) != -1) {
        switch (c) {
        case 'e':
            endless = true;
            filename = optarg;
            break;
        case 'o':
            outfile = optarg;
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

    /** Decide on output stream. */
    FILE *stream = stdout;
    if (outfile != NULL) {
        stream = fopen(outfile, "w");
        if (stream == NULL) {
            fprintf(stderr, "Could not open file '%s' for reading.\n", outfile);
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

    /* Print out the board ID EEPROM contents. */
    uint8_t const *board_id = eeprom_contents(bus);

    /* Create sensor data collection threads. */

    /* Sysclock. */
    pthread_t sysclock;
    collector_args_t sysclock_args = {.bus = bus, .addr = 0x00};
    errno_t err = pthread_create(&sysclock, NULL, &sysclock_collector, &sysclock_args);
    if (err != EOK) {
        fprintf(stderr, "Could not create sysclock thread: %s\n", strerror(err));
        exit(EXIT_FAILURE);
    }

    /* MS5611 */
    pthread_t ms5611;
    collector_args_t ms5611_args = {.bus = bus, .addr = 0x77};
    err = pthread_create(&ms5611, NULL, &ms5611_collector, &ms5611_args);
    if (err != EOK) {
        fprintf(stderr, "Could not create MS5611 thread: %s\n", strerror(err));
        exit(EXIT_FAILURE);
    }

    /* SHT41 */
    pthread_t sht41;
    collector_args_t sht41_args = {.bus = bus, .addr = 0x44};
    err = pthread_create(&sht41, NULL, &sht41_collector, &sht41_args);
    if (err != EOK) {
        fprintf(stderr, "Could not create SHT41 thread: %s\n", strerror(err));
        exit(EXIT_FAILURE);
    }

    /* LSM6DSO32 */
    pthread_t lsm6dso32;
    collector_args_t lsm6dso32_args = {.bus = bus, .addr = 0x6B};
    err = pthread_create(&lsm6dso32, NULL, &lsm6dso32_collector, &lsm6dso32_args);
    if (err != EOK) {
        fprintf(stderr, "Could not create LSM6DSO32 thread: %s\n", strerror(err));
        exit(EXIT_FAILURE);
    }

    // Wait for threads
    pthread_join(sysclock, NULL);
    pthread_join(ms5611, NULL);
    pthread_join(sht41, NULL);
    pthread_join(lsm6dso32, NULL);

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
