#include "sensors/ms5611/ms5611.h"
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

    uint8_t context_data[ms5611.context.size];
    ms5611.context.data = context_data;
    errno_t setup_res = ms5611.open(&ms5611);
    if (setup_res != EOK) {
        fprintf(stderr, "%s\n", strerror(setup_res));
        exit(EXIT_FAILURE);
    }

    // Read temperature and pressure data
    while (!endless) {
        uint8_t nbytes;
        float data;
        errno_t read_res = ms5611.read(&ms5611, TAG_TEMPERATURE, (uint8_t *)&data, &nbytes);
        if (read_res != EOK) {
            fprintf(stderr, "Could not read MS5611 temp: %s\n", strerror(read_res));
        }
        printf("Temperature: %2f C\n", data);
        read_res = ms5611.read(&ms5611, TAG_PRESSURE, (uint8_t *)&data, &nbytes);
        if (read_res != EOK) {
            fprintf(stderr, "Could not read MS5611 pressure: %s\n", strerror(read_res));
        }
        printf("Pressure: %.2f kPa\n", data);
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
