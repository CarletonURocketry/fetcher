#ifndef _COLLECTORS_H_
#define _COLLECTORS_H_

#include <mqueue.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** The name of the message queue to be used for sensors to write their data. */
#define SENSOR_QUEUE "fetcher/sensors"

/** Macro for dereferencing the collector argument. */
#define clctr_args(args) ((collector_args_t *)((args)))

typedef void *(*collector_t)(void *);

/** Collector name + thread entries for associating control threads with sensor names. */
typedef struct {
    const char *name;            /**< The name of the sensor associated with the collector thread. */
    const collector_t collector; /**< The function pointer to the collector thread. */
} clctr_entry_t;

/** Arguments for sensor threads. */
typedef struct {
    int bus;      /**< The I2C bus file descriptor. */
    uint8_t addr; /**< The address of the device on the I2C bus. */
} collector_args_t;

collector_t collector_search(const char *sensor_name);

/* Collector threads */
void *sysclock_collector(void *args);
void *ms5611_collector(void *args);
void *sht41_collector(void *args);
void *lsm6dso32_collector(void *args);
void *m10spg_collector(void *args);

#endif // _COLLECTORS_H_
