/**
 * @file main.c
 * @brief The main function for the fetcher module, where program logic is used to create a console application.
 *
 * The main function for the fetcher module, where program logic is used to create a console application.
 */
#include "board-id/board_id.h"
#include "collectors/collectors.h"
#include "drivers/m24c02/m24c02.h"
#include "sensor_api.h"
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
#define BUS_SPEED 100000

/** The maximum number of addresses per sensor type. */
#define MAX_ADDR_PER_SENSOR 5

/** The maximum number of characters in a sensor identifier. */
#define MAX_SENSOR_NAME 20

/** The maximum number of sensors that fetcher can support. */
#define MAX_SENSORS 8

#define SYSCLOCK_NAME "sysclock"

/** Whether or not to print data to stdout. */
bool print_output = false;

/** The name of a single sensor to enable, or null if no sensor was selected */
char *select_sensor = NULL;

/** Stores the thread IDs of all the collector threads. */
static pthread_t collector_threads[MAX_SENSORS];

/** Stores the collector arguments of all the collector threads. */
static collector_args_t collector_args[MAX_SENSORS];

/** Buffer for reading sensor messages when print option is selected. */
uint8_t buffer[BUFFER_SIZE];

/** Device descriptor of the I2C bus. */
char *i2c_bus = NULL;

/** A buffer for the contents of the board ID EEPROM. */
char board_id[M24C02_CAP];

int main(int argc, char **argv) {

    int c; // Holder for choice
    opterr = 0;

    /* Get command line options. */
    while ((c = getopt(argc, argv, ":ps:")) != -1) {
        switch (c) {
        case 'p':
            print_output = true;
            break;
        case 's':
            select_sensor = optarg;
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

    /* Positional argument for device descriptor. */
    if (optind >= argc) {
        fprintf(stderr, "I2C bus device descriptor is required.\n");
        exit(EXIT_FAILURE);
    }
    i2c_bus = argv[optind];

    /*
     * Open/create the message queue.
     * Main thread can only read incoming messages from sensors.
     * Other threads (collectors) can only write.
     */
    struct mq_attr q_attr = {
        .mq_flags = 0,
        .mq_maxmsg = 30,
        .mq_msgsize = 50,
    };
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_CREAT | O_RDONLY, S_IWOTH | S_IRUSR, &q_attr);
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
    int bus = open(i2c_bus, O_RDWR);
    if (bus < 0) {
        fprintf(stderr, "Could not open I2C bus with error %s.\n", strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* Set I2C bus speed. */
    uint32_t speed = BUS_SPEED;
    int err = devctl(bus, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof(speed), NULL);
    if (err != EOK) {
        fprintf(stderr, "Failed to set bus speed to %u with error %s\n", speed, strerror(err));
        exit(EXIT_FAILURE);
    }

    /* Parse the board ID EEPROM contents and configure drivers. */
    SensorLocation eeprom_loc = {.bus = bus, .addr = {.addr = BOARD_ID_ADDR, .fmt = I2C_ADDRFMT_7BIT}};
    err = m24c02_seq_read_rand(&eeprom_loc, 0x00, (uint8_t *)board_id, sizeof(board_id));
    if (err) {
        fprintf(stderr, "Failed to read EEPROM configuration.\n");
        exit(EXIT_FAILURE);
    }
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

        // Check if the sensor that was requested is this sensor (if none match, will do nothing)
        if (select_sensor != NULL) {
            if (strncasecmp(select_sensor, sensor_name, MAX_SENSOR_NAME) != 0) {
                // Skip this sensor, not the right one
                fprintf(stderr, "Skipping sensor %s\n", sensor_name);
                continue;
            } else {
                fprintf(stderr, "Found sensor %s, starting\n", select_sensor);
            }
        }
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
    // Only start the sysclock if we're not debugging a single sensor or if this is the sensor that was selected
    if (select_sensor == NULL || !strncasecmp(select_sensor, SYSCLOCK_NAME, sizeof(SYSCLOCK_NAME))) {
        /* Add sysclock sensor because it won't be specified in board ID. */
        collector_t sysclock = collector_search(SYSCLOCK_NAME);
        err = pthread_create(&collector_threads[num_sensors], NULL, sysclock, NULL);
        num_sensors++;
    }

    /* Constantly receive from sensors on message queue and print data. */
    while (print_output) {
        if (mq_receive(sensor_q, (char *)buffer, sensor_q_attr.mq_msgsize, NULL) == -1) {
            // Handle error without exiting
            fprintf(stderr, "Failed to receive message on queue '%s': %s\n", SENSOR_QUEUE, strerror(errno));
            continue;
        }
        // Successfully received data, print it to output stream
        sensor_write_data(stdout, buffer[0], &buffer[1]);
    }

    /* Wait for collectors to terminate before terminating. */
    for (uint8_t i = 0; i < num_sensors; i++) {
        pthread_join(collector_threads[i], NULL);
    }

    return 0;
}
