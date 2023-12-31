/**
 * @file sensor_api.c
 * @brief This file contains the implementations for the sensor API interface.
 *
 * This file contains the implementations for the sensor API interface.
 */
#include "sensor_api.h"
#include <stdio.h>
#include <string.h>

/** Macro to cast a pointer to a different type and dereferencing it. */
#define drefcast(type, data) (*((type *)data))

/** A list of the possible sensor tags and their metadata. */
const SensorTagData SENSOR_TAG_DATA[] = {
    [TAG_PRESSURE] =
        {.name = "Pressure", .unit = "kPa", .fmt_str = "%.2f", .dsize = sizeof(float), .dtype = TYPE_FLOAT},
    [TAG_TEMPERATURE] =
        {.name = "Temperature", .unit = "C", .fmt_str = "%.2f", .dsize = sizeof(float), .dtype = TYPE_FLOAT},
    [TAG_TIME] = {.name = "Time", .unit = "ms", .fmt_str = "%u", .dsize = sizeof(uint32_t), .dtype = TYPE_U32},
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
 * Gets the maximum dsize value of the tags in the tag list.
 * @param tag_list The tag list to get the maximum dsize of.
 * @return The maximum dsize of all the tags in the tag list.
 */
uint8_t sensor_max_dsize(const SensorTagList *tag_list) {
    uint8_t max = 0;
    for (uint8_t i = 0; i < tag_list->len; i++) {
        if (tag_list->tags[i] > max) {
            max = SENSOR_TAG_DATA[tag_list->tags[i]].dsize;
        }
    }
    return max;
}

/**
 * Converts a tag into the string representation of its associated data type's name.
 * @param tag The tag to stringify.
 * @return The tag's data type as a string.
 */
const char *sensor_strtag(const SensorTag tag) { return SENSOR_TAG_DATA[tag].unit; }

/**
 * Prints sensor data in a standard format.
 * @param tag The tag describing the kind of sensor data.
 * @param data A pointer to the sensor data to be printed.
 */
void sensor_print_data(const SensorTag tag, const void *data) {
    char format_str[40] = "%s: ";                     // Format specifier for data name
    strcat(format_str, SENSOR_TAG_DATA[tag].fmt_str); // Format specifier for data
    strcat(format_str, " %s\n");                      // Format specifier for unit

    // Ignore GCC warning about not using string literal in printf
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    // Decide how to cast data pointer for printing
    switch (SENSOR_TAG_DATA[tag].dtype) {
    case TYPE_FLOAT:
        printf(format_str, SENSOR_TAG_DATA[tag].name, drefcast(const float, data), SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_U32:
        printf(format_str, SENSOR_TAG_DATA[tag].name, drefcast(const uint32_t, data), SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_U16:
        printf(format_str, SENSOR_TAG_DATA[tag].name, drefcast(const uint16_t, data), SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_U8:
        printf(format_str, SENSOR_TAG_DATA[tag].name, drefcast(const uint8_t, data), SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_I32:
        printf(format_str, SENSOR_TAG_DATA[tag].name, drefcast(const int32_t, data), SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_I16:
        printf(format_str, SENSOR_TAG_DATA[tag].name, drefcast(const int16_t, data), SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_I8:
        printf(format_str, SENSOR_TAG_DATA[tag].name, drefcast(const int8_t, data), SENSOR_TAG_DATA[tag].unit);
        break;
    default:
        return;
    }
}
