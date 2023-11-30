/**
 * @file sensor_api.c
 * @brief This file contains the implementations for the sensor API interface.
 *
 * This file contains the implementations for the sensor API interface.
 */
#include "sensor_api.h"

/** A list of the possible sensor tags and their string representation. */
static const char *TAG_STRINGS[] = {
    [TAG_PRESSURE] = "Pressure",
    [TAG_TEMPERATURE] = "Temperature",
    [TAG_TIME] = "Time",
};

/** A list of the possible sensor tags and their units in string representation. */
static const char *TAG_UNITS[] = {
    [TAG_PRESSURE] = "kPa",
    [TAG_TEMPERATURE] = "C",
    [TAG_TIME] = "s",
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
 * Converts a sensor tag to its string representation.
 * @param tag The sensor tag to stringify.
 * @return The sensor tag string representation.
 */
const char *sensor_strtag(const SensorTag tag) { return TAG_STRINGS[tag]; }

/**
 * Converts a sensor tag to its unit in string representation.
 * @param tag The sensor tag to its unit in stringify.
 * @return The sensor tag unit in string representation.
 */
const char *sensor_tag_unit(const SensorTag tag) { return TAG_UNITS[tag]; }
