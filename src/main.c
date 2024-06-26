/**
 * @file main.c
 * @brief The main function for the fetcher module, where program logic is used to create a console application.
 *
 * The main function for the fetcher module, where program logic is used to create a console application.
 */
#include "../logging-utils/logging.h"
#include "board-id/board_id.h"
#include "collectors/collectors.h"
#include "drivers/m24c0x/m24c0x.h"
#include "drivers/sensor_api.h"
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

/** The name of the system clock collector. */
#define SYSCLOCK_NAME "sysclock"

/** Whether or not to print data to stdout. */
bool print_output = false;

/** The name of a single sensor to enable, or null if no sensor was selected */
char *select_sensor = NULL;

/** Stores the thread IDs of all the collector threads. */
pthread_t collector_threads[MAX_SENSORS];

/** Stores the collector arguments of all the collector threads. */
collector_args_t collector_args[MAX_SENSORS];

/** Space for recieving messages from the message queue if print mode is selected. */
static common_t recv_msg;

/** Device descriptor of the I2C bus. */
char *i2c_bus = NULL;

/** A buffer for the contents of the board ID EEPROM. */
char board_id[M24C02_CAP + 1] = {0};

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
        .mq_msgsize = sizeof(recv_msg),
    };
    mqd_t sensor_q = mq_open(SENSOR_QUEUE, O_CREAT | O_RDONLY, S_IWOTH | S_IRUSR, &q_attr);
    if (sensor_q == -1) {
        log_print(stderr, LOG_ERROR, "Could not create internal queue '%s' with error: '%s'", SENSOR_QUEUE,
                  strerror(errno));
        exit(EXIT_FAILURE);
    }

    // Get message queue attributes since it's necessary to know max message size for receiving
    struct mq_attr sensor_q_attr;
    if (mq_getattr(sensor_q, &sensor_q_attr) == -1) {
        log_print(stderr, LOG_ERROR, "Failed to get attributes of message queue '%s': '%s'", SENSOR_QUEUE,
                  strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* Open I2C. */
    int bus = open(i2c_bus, O_RDWR);
    if (bus < 0) {
        log_print(stderr, LOG_ERROR, "Could not open I2C bus with error %s.", strerror(errno));
        exit(EXIT_FAILURE);
    }

    /* Set I2C bus speed. */
    uint32_t speed = BUS_SPEED;
    int err = devctl(bus, DCMD_I2C_SET_BUS_SPEED, &speed, sizeof(speed), NULL);
    if (err) {
        log_print(stderr, LOG_ERROR, "Failed to set bus speed to %u with error %s", speed, strerror(err));
        exit(EXIT_FAILURE);
    }

    /* Parse the board ID EEPROM contents and configure drivers. */
    SensorLocation eeprom_loc = {.bus = bus, .addr = {.addr = BOARD_ID_ADDR, .fmt = I2C_ADDRFMT_7BIT}};
    err = m24c0x_seq_read_rand(&eeprom_loc, 0x00, (uint8_t *)board_id, M24C02_CAP);
    if (err) {
        log_print(stderr, LOG_ERROR, "Failed to read EEPROM configuration: %s", strerror(err));
        exit(EXIT_FAILURE);
    }
    board_id[M24C02_CAP] = '\0'; // Make sure the string ends with a null terminator
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
                log_print(stderr, LOG_ERROR, "Skipping sensor %s", sensor_name);
                continue;
            } else {
                log_print(stderr, LOG_INFO, "Found sensor %s, starting...", select_sensor);
            }
        }
        for (uint8_t i = 0; i < naddrs; i++) {
            /* Create sensor data collection threads. */
            collector_t collector = collector_search(sensor_name);
            if (collector == NULL) {
                log_print(stderr, LOG_ERROR, "Collector not implemented for sensor %s", sensor_name);
                continue; // Just don't create thread
            }
            collector_args[num_sensors] = (collector_args_t){.bus = bus, .addr = addresses[i]};
            err = pthread_create(&collector_threads[num_sensors], NULL, collector, &collector_args[num_sensors]);
            if (err != EOK) {
                log_print(stderr, LOG_ERROR, "Could not create %s collector: %s", sensor_name, strerror(err));
                exit(EXIT_FAILURE);
            }
            num_sensors++; // Record that a new sensor was created
        }
    }
    // Only start the sysclock if we're not debugging a single sensor or if this is the sensor that was selected
    if (select_sensor == NULL || !strcasecmp(select_sensor, SYSCLOCK_NAME)) {
        /* Add sysclock sensor because it won't be specified in board ID. */
        collector_t sysclock = collector_search(SYSCLOCK_NAME);
        err = pthread_create(&collector_threads[num_sensors], NULL, sysclock, NULL);
        num_sensors++;
    }

    /* Add PAC1952 sensor because it won't be specified in board ID. */
    if (select_sensor == NULL || !strcasecmp(select_sensor, "pac1952-2")) {
        collector_t pac1952 = collector_search("pac1952-2");
        collector_args[num_sensors] = (collector_args_t){.bus = bus, .addr = 0x17};
        err = pthread_create(&collector_threads[num_sensors], NULL, pac1952, &collector_args[num_sensors]);
        num_sensors++;
    }

    /* Constantly receive from sensors on message queue and print data. */
    while (print_output) {
        if (mq_receive(sensor_q, (char *)&recv_msg, sensor_q_attr.mq_msgsize, NULL) == -1) {
            // Handle error without exiting
            log_print(stderr, LOG_ERROR, "Failed to receive message on queue '%s': %s", SENSOR_QUEUE, strerror(errno));
            continue;
        }
        // Successfully received data, print it to output stream
        sensor_write_data(stdout, &recv_msg);
    }

    /* Wait for collectors to terminate before terminating. */
    for (uint8_t i = 0; i < num_sensors; i++) {
        pthread_join(collector_threads[i], NULL);
    }

    return 0;
}
