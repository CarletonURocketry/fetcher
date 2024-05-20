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
    [TAG_HUMIDITY] =
        {.name = "Humidity", .unit = "%RH", .fmt_str = "%.2f", .dsize = sizeof(float), .dtype = TYPE_FLOAT},
    [TAG_TIME] = {.name = "Time", .unit = "ms", .fmt_str = "%u", .dsize = sizeof(uint32_t), .dtype = TYPE_U32},
    [TAG_ALTITUDE] = {.name = "Altitude", .unit = "m", .fmt_str = "%.2f", .dsize = sizeof(float), .dtype = TYPE_FLOAT},
    [TAG_LINEAR_ACCEL] = {.name = "Linear acceleration",
                          .unit = "m/s^2",
                          .fmt_str = "%.2fX, %.2fY, %.2fZ",
                          .dsize = sizeof(vec3d_t),
                          .dtype = TYPE_VEC3D},
    [TAG_ANGULAR_VEL] = {.name = "Angular velocity",
                         .unit = "dps",
                         .fmt_str = "%.2fX, %.2fY, %.2fZ",
                         .dsize = sizeof(vec3d_t),
                         .dtype = TYPE_VEC3D},
    [TAG_LONGITUDE] =
        {.name = "Longitude", .unit = "0.1udeg", .fmt_str = "%d", .dsize = sizeof(int32_t), .dtype = TYPE_I32},
    [TAG_LATITUDE] =
        {.name = "Latitude", .unit = "0.1udeg", .fmt_str = "%d", .dsize = sizeof(int32_t), .dtype = TYPE_I32},
    [TAG_ALTITUDE_MSL] =
        {.name = "Altitude (MSL)", .unit = "mm", .fmt_str = "%d", .dsize = sizeof(int32_t), .dtype = TYPE_I32},
    [TAG_SPEED] =
        {.name = "Ground speed", .unit = "cm/s", .fmt_str = "%d", .dsize = sizeof(uint32_t), .dtype = TYPE_U32},
    [TAG_COURSE] = {.name = "Course", .unit = "10udeg", .fmt_str = "%d", .dsize = sizeof(uint32_t), .dtype = TYPE_U32},
    [TAG_FIX] = {.name = "Fix type", .unit = "", .fmt_str = "0x%x", .dsize = sizeof(uint8_t), .dtype = TYPE_U8},
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
 * Sets the precision of the sensor to the desired value.
 * @param sensor The sensor to set precision for.
 * @param precision The precision to set the sensor to.
 */
#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
inline void __attribute__((always_inline)) sensor_set_precision(Sensor sensor, const SensorPrecision precision) {
    sensor.precision = precision;
}

/**
 * Gets the size of the sensor's context.
 * @param sensor The sensor to get the context size of.
 * @return The size of the sensor context.
 */
#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
inline size_t __attribute__((always_inline)) sensor_get_ctx_size(Sensor sensor) { return sensor.context.size; }

/**
 * Sets the sensor context to the memory block pointed to by buf.
 * @param sensor The sensor to set the context for.
 * @param buf The memory block to set the sensor context to.
 */
#pragma GCC diagnostic ignored "-Wunused-but-set-parameter"
inline void __attribute__((always_inline)) sensor_set_ctx(Sensor *sensor, void *buf) { sensor->context.data = buf; }

/**
 * Open and set up the sensor.
 * @param sensor The sensor to open and set up.
 * @return Error status of setting up the sensor. EOK if successful.
 */
inline errno_t __attribute__((always_inline)) sensor_open(Sensor sensor) { return sensor.open(&sensor); }

/**
 * Read the specified data from the sensor.
 * @param sensor The sensor to read from.
 * @param tag The sensor tag (data type) to read from the sensor.
 * @param buf The memory location to store the result of the read.
 * @param nbytes Will be populated with the number of bytes that were stored in buf after the read.
 * @return Error status of setting up the sensor. EOK if successful.
 */
inline errno_t __attribute__((always_inline))
sensor_read(Sensor sensor, const SensorTag tag, void *buf, size_t *nbytes) {
    return sensor.read(&sensor, tag, buf, nbytes);
}

/**
 * Gets the maximum data size returned when reading the sensor.
 * @param sensor The sensor to find the maximum data size of.
 * @return The maximum dsize of all the tags in the tag list.
 */
size_t sensor_max_dsize(const Sensor *sensor) {
    uint8_t max = 0;
    for (uint8_t i = 0; i < sensor->tag_list.len; i++) {
        if (sensor->tag_list.tags[i] > max) {
            max = SENSOR_TAG_DATA[sensor->tag_list.tags[i]].dsize;
        }
    }
    return max;
}

/**
 * Converts a tag into the string representation of its associated data type's name.
 * @param tag The tag to stringify.
 * @return The tag's data type as a string.
 */
const char __attribute__((const)) * sensor_strtag(const SensorTag tag) { return SENSOR_TAG_DATA[tag].name; }

/**
 * Writes sensor data in a standard format.
 * @param stream The output stream for writing sensor data.
 * @param tag The tag describing the kind of sensor data.
 * @param data A pointer to the sensor data to be printed.
 */
void sensor_write_data(FILE *stream, const SensorTag tag, const void *data) {
    char format_str[40] = "%s: ";                     // Format specifier for data name
    strcat(format_str, SENSOR_TAG_DATA[tag].fmt_str); // Format specifier for data
    strcat(format_str, " %s\n");                      // Format specifier for unit

    // Ignore GCC warning about not using string literal in printf
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    // Decide how to cast data pointer for printing
    switch (SENSOR_TAG_DATA[tag].dtype) {
    case TYPE_FLOAT:
        // Ignore warning about promoting float to double since printf doesn't support float printing
#pragma GCC diagnostic ignored "-Wdouble-promotion"
        fprintf(stream, format_str, SENSOR_TAG_DATA[tag].name, drefcast(const float, data), SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_U32:
        fprintf(stream, format_str, SENSOR_TAG_DATA[tag].name, drefcast(const uint32_t, data),
                SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_U16:
        fprintf(stream, format_str, SENSOR_TAG_DATA[tag].name, drefcast(const uint16_t, data),
                SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_U8:
        fprintf(stream, format_str, SENSOR_TAG_DATA[tag].name, drefcast(const uint8_t, data),
                SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_I32:
        fprintf(stream, format_str, SENSOR_TAG_DATA[tag].name, drefcast(const int32_t, data),
                SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_I16:
        fprintf(stream, format_str, SENSOR_TAG_DATA[tag].name, drefcast(const int16_t, data),
                SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_I8:
        fprintf(stream, format_str, SENSOR_TAG_DATA[tag].name, drefcast(const int8_t, data), SENSOR_TAG_DATA[tag].unit);
        break;
    case TYPE_VEC3D:
        fprintf(stream, format_str, SENSOR_TAG_DATA[tag].name, drefcast(const vec3d_t, data).x,
                drefcast(const vec3d_t, data).y, drefcast(const vec3d_t, data).z, SENSOR_TAG_DATA[tag].unit);
        break;
    }
}
