/**
 * @file sensor_api.c
 * @brief This file contains the implementations for the sensor API interface.
 *
 * This file contains the implementations for the sensor API interface.
 */
#include "sensor_api.h"
#include <stdio.h>
#include <string.h>

/** Stores information about each tag. */
typedef struct {
    const char *name;
    const char *unit;
    const char *fmt_str;
} TagData;

/** A list of the possible sensor tags and their metadata. */
static const TagData TAG_DATA[] = {
    [TAG_PRESSURE] = {.name = "Pressure", .unit = "kPa", .fmt_str = "%p"},
    [TAG_TEMPERATURE] = {.name = "Temperature", .unit = "C", .fmt_str = "%p"},
    [TAG_TIME] = {.name = "Time", .unit = "ms", .fmt_str = "%p"},
};

/**
 * Utility function for copying memory in big-endian format.
 * @param dest The destination buffer for data copied from src.
 * @param src The source buffer for data to be copied into dest.
 * @param nbytes The number of bytes to copy from src into dest.
 */
void memcpy_be(void *dest, const void *src, const size_t nbytes) {
    for (size_t i = nbytes; i > 0; i--) {
        ((uint8_t *)dest)[i - 1] = ((const uint8_t *)src)[nbytes - i];
    }
}

/**
 * Converts a tag into the string representation of its associated data type's name.
 * @param tag The tag to stringify.
 * @return The tag's data type as a string.
 */
const char *sensor_strtag(const SensorTag tag) { return TAG_DATA[tag].unit; }

/**
 * Prints sensor data in a standard format.
 * @param tag The tag describing the kind of sensor data.
 * @param data A pointer to the sensor data to be printed.
 */
void sensor_print_data(const SensorTag tag, const void *data) {
    char format_str[40] = "%s: ";              // Format specifier for data name
    strcat(format_str, TAG_DATA[tag].fmt_str); // Format specifier for data
    strcat(format_str, " %s\n");               // Format specifier for unit

// Ignore GCC warning just for this line
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    printf(format_str, TAG_DATA[tag].name, data, TAG_DATA[tag].unit);
}
