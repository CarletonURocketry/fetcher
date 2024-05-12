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

/** The speed of the I2C bus in kilobits per second. */
#define BUS_SPEED 400000

/** The maximum number of addresses per sensor type. */
#define MAX_ADDR_PER_SENSOR 5

/** The maximum number of characters in a sensor identifier. */
#define MAX_SENSOR_NAME 20

/** The maximum number of sensors that fetcher can support. */
#define MAX_SENSORS 8

/** Stores the thread IDs of all the collector threads. */
static pthread_t collector_threads[MAX_SENSORS];

/** Stores the collector arguments of all the collector threads. */
static collector_args_t collector_args[MAX_SENSORS];

/** Flag to indicate reading from file in endless mode for debugging. */
static bool endless = false;

/** Name of file to read from, if one is provided. */
static char *filename = NULL;

/** Name of file to write to, if one is provided. */
static char *outfile = NULL;

/** Buffer for reading data. */
static char buffer[BUFFER_SIZE];

/**
 * Reads the sensor name from the board ID into a new buffer.
 * @param board_id The contents of the board ID.
 * @param sensor_name The buffer to store the sensor name.
 * @param nbytes The maximum number of bytes to read.
 * @return The current position inside the board ID after reading, or NULL if nbytes needs to be larger.
 */
const char *read_sensor_name(const char *board_id, char *sensor_name, uint8_t nbytes) {
    uint8_t pos;
    for (pos = 0; *board_id != ' ' && *board_id != '\0' && pos < nbytes; pos++, board_id++) {
        sensor_name[pos] = *board_id;
    }

    if (pos == nbytes) return NULL; // Could not successfully read the contents

    // We didn't hit end of content, skip newline character
    if (*board_id != '\0') board_id++;

    sensor_name[pos] = '\0';
    return board_id;
}

/**
 * Reads up to `naddrs` addresses from the board ID into an array of `addresses`.
 * @param board_id The current position in the board ID where an address starts.
 * @param addresses An array of bytes with enough space to store `naddrs` bytes.
 * @param naddrs A pointer to the maximum number of addresses to read. After the function call, this pointer will
 * contain the actual number of addresses read.
 * @return The current position in the board ID after the read, or NULL if `naddrs` needs to be larger.
 */
const char *read_sensor_addresses(const char *board_id, uint8_t *addresses, uint8_t *naddrs) {

    uint8_t num_addrs;
    for (num_addrs = 0; *board_id != '\n' && *board_id != '\0' && num_addrs < *naddrs; board_id++) {
        if (*board_id == ' ') {
            // Address is hex with two digits, so two digits back from space will be address start
            // Convert this hex into an actual numerical byte
            addresses[num_addrs] = strtoul(board_id - 2, NULL, 16);
            num_addrs++;
        }
    }

    if (num_addrs == *naddrs) return NULL; // Need to be able to read more addresses

    // We hit end of content
    if (*board_id == '\0') {
        *naddrs = num_addrs;
        return board_id;
    }

    // Hit a newline, get last address (three positions back because board_id was incremented an extra time exiting the
    // for loop)
    addresses[num_addrs] = strtoul(board_id - 3, NULL, 16);
    num_addrs++;
    board_id++; // Skip over final newline character

    *naddrs = num_addrs;
    return board_id;
}

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

    /*
     * Open/create the message queue.
     * Main thread can only read incoming messages from sensors.
     * Other threads (collectors) can only write.
     */
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_CREAT | O_RDONLY, S_IWOTH | S_IRUSR, NULL);
    if (sensor_q == -1) {
        fprintf(stderr, "Could not create internal queue '%s' with error: '%s'\n", SENSOR_QUEUE, strerror(errno));
        exit(EXIT_FAILURE);
    }

    // Get message queue attributes since it's necessary to know max message size for receiving
    struct mq_attr sensor_q_attr;
    if (mq_getattr(sensor_q, &sensor_q_attr) == -1) {
        fprintf(stderr, "Failed to get attributes of message queue '%s': '%s'\n", SENSOR_QUEUE, strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* Open I2C. */
    int bus = open("/dev/i2c1", O_RDWR);
    if (bus < 0) {
        fprintf(stderr, "Could not open I2C bus with error %s.\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* Set I2C bus speed. */
    uint32_t speed = BUS_SPEED;
    errno_t err = devctl(bus, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof(speed), NULL);
    if (err != EOK) {
        fprintf(stderr, "Failed to set bus speed to %u with error %s\n", speed, strerror(err));
        exit(EXIT_FAILURE);
    }

    /* Parse the board ID EEPROM contents and configure drivers. */
    char const *board_id = (const char *)eeprom_contents(bus);
    const char *cur = board_id;

    // Skip the first two lines (board ID and CU InSpace credit)
    uint8_t lines = 0;
    while (lines < 2) {
        if (*cur == '\n') lines++;
        cur++;
    }

    // Parse each sensor line
    uint8_t num_sensors = 0;
    while (*cur != '\0') {

        char sensor_name[MAX_SENSOR_NAME];
        cur = read_sensor_name(cur, sensor_name, 20);

        // Get sensor addresses
        uint8_t addresses[MAX_ADDR_PER_SENSOR];
        uint8_t naddrs = MAX_ADDR_PER_SENSOR;
        cur = read_sensor_addresses(cur, addresses, &naddrs);

        for (uint8_t i = 0; i < naddrs; i++) {

            /* Create sensor data collection threads. */
            collector_t collector = collector_search(sensor_name);
            if (collector == NULL) {
                fprintf(stderr, "Collector not implemented for sensor %s\n", sensor_name);
                continue; // Just don't create thread
            }
            collector_args[num_sensors] = (collector_args_t){.bus = bus, .addr = addresses[i]};
            err = pthread_create(&collector_threads[num_sensors], NULL, collector, &collector_args[num_sensors]);
            if (err != EOK) {
                fprintf(stderr, "Could not create %s collector: %s\n", sensor_name, strerror(err));
                exit(EXIT_FAILURE);
            }
            num_sensors++; // Record that a new sensor was created
        }
    }

    /* Add sysclock sensor because it won't be specified in board ID. */
    collector_t sysclock = collector_search("sysclock");
    collector_args[num_sensors] = (collector_args_t){.bus = bus, .addr = 0x00};
    err = pthread_create(&collector_threads[num_sensors], NULL, sysclock, &collector_args[num_sensors]);

    /* Constantly receive from sensors on message queue and print data. */
    while (!endless) {
        mq_receive(sensor_q, buffer, sensor_q_attr.mq_msgsize, NULL);
        printf("%s", buffer);
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

    /* Wait for collectors to terminate before terminating. */
    for (uint8_t i = 0; i < num_sensors; i++) {
        pthread_join(collector_threads[i], NULL);
    }

    return 0;
}
