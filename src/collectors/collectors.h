#ifndef _COLLECTORS_H_
#define _COLLECTORS_H_

#include <mqueue.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Macro for dereferencing the collector argument. */
#define clctr_args(args) ((collector_args_t*)((args)))

/** Arguments for sensor threads. */
typedef struct {
    int bus;      /**< The I2C bus file descriptor. */
    uint8_t addr; /**< The address of the device on the I2C bus. */
} collector_args_t;

/* Collector threads */
void *sysclock_collector(void *args);
void *ms5611_collector(void *args);

#endif // _COLLECTORS_H_
