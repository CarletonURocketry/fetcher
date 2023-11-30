#include "sysclock.h"
#include "../sensor_api.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

/** The types of measurements the system clock can perform. */
static const SensorTag TAGS[] = {TAG_TIME};

/** The structure containing the context the system clock needs between reads. */
typedef struct {
    /** The offset of the current timezone from the GMT timezone. */
    long int tm_gmtoff;
    /** The start time (when the sensor was opened) in Unix time. */
    time_t start_unix_time;
} SysClockContext;

/** Macro for easily dereferencing the system clock context. */
#define derefctx(ctx) (*((SysClockContext *)ctx.data))

/**
 * Prepares the system clock for reading.
 * @param sensor A reference to a system clock sensor.
 * @return The error status of the call. EOK if successful.
 */
static errno_t sysclock_open(Sensor *sensor) {

    // Get the offset from the GMT timezone
    time_t unix_time;
    time(&unix_time);
    struct tm *time_info = localtime(&unix_time);

    // Store the GMT offset and start time in the context
    SysClockContext context = {.tm_gmtoff = time_info->tm_gmtoff, .start_unix_time = unix_time};
    memcpy(sensor->context.data, &context, sizeof(context));
    return EOK;
}

/**
 * Reads the specified data from the system clock.
 * @param sensor A reference to a system clock sensor.
 * @param tag The tag of the data type that should be read.
 * @param buf A pointer to the byte array to store the data.
 * @param nbytes The number of bytes that were written into the byte array buffer.
 * @return Error status of reading from the sensor. EOK if successful.
 */
static errno_t sysclock_read(Sensor *sensor, const SensorTag tag, uint8_t *buf, uint8_t *nbytes) {

    (void)sensor; // Avoid unused parameter error

    switch (tag) {
    case TAG_TIME: {

        // Set timezone, no daylight savings correct and with the correct GMT offset
        struct timezone tz = {.tz_dsttime = 0, .tz_minuteswest = derefctx(sensor->context).tm_gmtoff};
        struct timeval tval;

        gettimeofday(&tval, &tz);
        time_t elapsed_s = tval.tv_sec - derefctx(sensor->context).start_unix_time;

        // Convert to milliseconds
        uint32_t ms = (elapsed_s * 1000) + (tval.tv_usec / 1000);
        memcpy(buf, &ms, sizeof(ms));
        *nbytes = sizeof(ms);
        return EOK;
    } break;
    default:
        return EINVAL;
    }
}

/**
 * Initializes a sensor struct with the interface to interact with the system clock.
 * @param sensor The sensor interface to be initialized.
 * @param bus The file descriptor of the I2C bus.
 * @param addr The address of the system clock on the I2C bus (doesn't matter).
 * @param precision The precision to read measurements with.
 */
void sysclock_init(Sensor *sensor, const int bus, const uint8_t addr, const SensorPrecision precision) {
    sensor->precision = precision;
    sensor->loc = (SensorLocation){.bus = bus, .addr = {.addr = (addr & 0x7F), .fmt = I2C_ADDRFMT_7BIT}};
    sensor->max_return_size = sizeof(uint32_t);
    sensor->tag_list = (SensorTagList){.tags = TAGS, .len = sizeof(TAGS) / sizeof(SensorTag)};
    sensor->context.size = sizeof(long);
    sensor->open = &sysclock_open;
    sensor->read = &sysclock_read;
}
