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
#include <stdio.h>

/** Type for a 3 dimensional vector with x, y, z components. */
typedef struct {
    /** X component. */
    float x;
    /** Y component. */
    float y;
    /** Z component. */
    float z;
} vec3d_t;

/** Describes what data type the sensor is able to read. */
typedef enum {
    TAG_TEMPERATURE,   /**< Temperature in degrees Celsius */
    TAG_PRESSURE,      /**< Pressure in kilo Pascals */
    TAG_HUMIDITY,      /**< Humidity in % relative humidity */
    TAG_TIME,          /**< Time in milliseconds */
    TAG_ALTITUDE,      /**< Altitude in meters */
    TAG_ANGULAR_ACCEL, /**< Angular acceleration in TODO units */
    TAG_LINEAR_ACCEL,  /**< Linear acceleration in TODO units */
} SensorTag;

/** Describes the data type of the data associated with a tag. */
typedef enum {
    TYPE_FLOAT, /**< float */
    TYPE_U32,   /**< uint32_t */
    TYPE_U16,   /**< uint16_t */
    TYPE_U8,    /**< uint8_t */
    TYPE_I32,   /**< int32_t */
    TYPE_I16,   /**< int16_t */
    TYPE_I8,    /**< int8_t */
    TYPE_VEC3D, /**< vec3d_t */
} SensorTagDType;

/** Stores information about each tag. */
typedef struct {
    /** The name of the data the tag is associated with. */
    const char *name;
    /** The name of the unit the tag's measurements are measured in. */
    const char *unit;
    /** The format string used to print the data associated with the tag. */
    const char *fmt_str;
    /** The size of the data this tag is associated with. */
    const size_t dsize;
    /** The C data type this data is associated with. */
    const SensorTagDType dtype;
} SensorTagData;

typedef enum {
    PRECISION_HIGH, /**< High precision measurements */
    PRECISION_MED,  /**< Medium precision measurements */
    PRECISION_LOW,  /**< Low precision measurements */
} SensorPrecision;

/** Stores a list of tags (data types) that the sensor is capable of reading. */
typedef struct {
    /** A pointer to which tags the sensor can read. */
    const SensorTag *tags;
    /** The number of tags the sensor can read. */
    uint8_t len;
} SensorTagList;

/** Provides an interface for the sensor to store operations between context. */
typedef struct {
    /** The sensor context itself. 'size' bytes of memory for this must be allocated by the user. */
    void *data;
    /** The size of the sensor context data in bytes. */
    size_t size;
} SensorContext;

/** Provides an interface to describe the sensor location. */
typedef struct {
    /** The address of the sensor on the bus. */
    i2c_addr_t addr;
    /** The I2C bus file descriptor. */
    int bus;
} SensorLocation;

/** The generic interface to interact with all sensors. */
typedef struct sensor_t {
    /** The I2C address and bus of the sensor. */
    SensorLocation loc;
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
     * @param buf A pointer to the memory location to store the data. Should be large enough to fit the max_return_size
     * bytes.
     * @param nbytes The number of bytes that were written into the byte array buffer.
     * @return Error status of reading from the sensor. EOK if successful.
     */
    errno_t (*read)(struct sensor_t *sensor, const SensorTag tag, void *buf, size_t *nbytes);
} Sensor;

void memcpy_be(void *dest, const void *src, const size_t nbytes);
size_t sensor_max_dsize(const Sensor *sensor);
const char *sensor_strtag(const SensorTag tag);
void sensor_write_data(FILE *stream, const SensorTag tag, const void *data);

extern void sensor_set_precision(Sensor sensor, const SensorPrecision precision);
extern errno_t sensor_open(Sensor sensor);
extern errno_t sensor_read(Sensor sensor, const SensorTag tag, void *buf, size_t *nbytes);
extern size_t sensor_get_ctx_size(Sensor sensor);
extern void sensor_set_ctx(Sensor *sensor, void *buf);

#endif // _SENSOR_API_H
