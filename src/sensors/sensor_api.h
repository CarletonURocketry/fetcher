/**
 * @file sensor_api.h
 * @brief Types and function prototypes for the sensor API used to communicate with I2C bus sensors.
 *
 * Types and function prototypes for the sensor API used to communicate with I2C bus sensors. This API provides a way to
 * set up and read sensors easily. Sensors can read multiple data types in standard measurements. Sensors also have a
 * configurable precision.
 */
#ifndef _SENSOR_API_H
#define _SENSOR_API_H

#include <errno.h>
#include <hw/i2c.h>
#include <stdint.h>

/** Describes what data type the sensor is able to read. */
typedef enum sensor_tag_t {
    TAG_TEMPERATURE, /**< Temperature in degrees Celsius */
    TAG_PRESSURE,    /**< Pressure in kilo Pascals */
    TAG_TIME,        /**< Time in milliseconds */
} SensorTag;

typedef enum sensor_precision_t {
    PRECISION_HIGH, /**< High precision measurements */
    PRECISION_MED,  /**< Medium precision measurements */
    PRECISION_LOW,  /**< Low precision measurements */
} SensorPrecision;

/** Stores a list of tags (data types) that the sensor is capable of reading. */
typedef struct tag_list_t {
    /** A pointer to which tags the sensor can read. */
    const SensorTag *tags;
    /** The number of tags the sensor can read. */
    uint8_t len;
} SensorTagList;

/** Provides an interface for the sensor to store operations between context. */
typedef struct sensor_context_t {
    /** The sensor context itself. 'size' bytes of memory for this must be allocated by the user. */
    void *data;
    /** The size of the sensor context data in bytes. */
    size_t size;
} SensorContext;

/** Provides an interface to describe the sensor location. */
typedef struct sensor_loc_t {
    /** The address of the sensor on the bus. */
    i2c_addr_t addr;
    /** The I2C bus file descriptor. */
    int bus;
} SensorLocation;

/** The generic interface to interact with all sensors. */
typedef struct sensor_t {
    /** The I2C address and bus of the sensor. */
    SensorLocation loc;
    /** The maximum return size of any data that is read. */
    uint8_t max_return_size;
    /** A list of all the tags the sensor is capable of reading. */
    SensorTagList tag_list;
    /** Data structure for the sensor to store context it needs between operations. */
    SensorContext context;
    /** What precision the sensor should measure. Can be changed between reads. */
    SensorPrecision precision;
    /**
     * Function responsible for opening and setting up the sensor.
     * @param sensor The sensor who this open method belongs to.
     * @return Error status of setting up the sensor. EOK if successful.
     */
    errno_t (*open)(struct sensor_t *sensor);
    /**
     * Function responsible for reading the data associated with tag from the sensor.
     * @param sensor The sensor who this read method belongs to.
     * @param tag The tag of the data type that should be read.
     * @param buf A pointer to the byte array to store the data. Should be large enough to fit the max_return_size
     * bytes.
     * @param nbytes The number of bytes that were written into the byte array buffer.
     * @return Error status of reading from the sensor. EOK if successful.
     */
    errno_t (*read)(struct sensor_t *sensor, const SensorTag tag, uint8_t *buf, uint8_t *nbytes);
} Sensor;

void memcpy_be(void *dest, const void *src, const size_t nbytes);

const char *sensor_strtag(const SensorTag tag);
const char *sensor_tag_unit(const SensorTag tag);

#endif // _SENSOR_API_H
