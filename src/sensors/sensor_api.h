#ifndef _SENSOR_API_H
#define _SENSOR_API_H

#include <errno.h>
#include <hw/i2c.h>
#include <stdint.h>

/** Describes what data type the sensor is able to read. */
typedef enum sensor_tag_t {
    TAG_TEMPERATURE, /**< Temperature */
    TAG_PRESSURE,    /**< Pressure */
} SensorTag;

/** Stores a list of tags (data types) that the sensor is capable of reading. */
typedef struct tag_list_t {
    /** A pointer to which tags the sensor can read. */
    const SensorTag *tags;
    /** The number of tags the sensor can read. */
    uint8_t tag_count;
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
    /**
     * Function responsible for opening and setting up the sensor.
     * @param loc The location of the sensor on the I2C bus.
     * @param context A pointer to the sensor context required between operations. Size bytes must be allocated in the
     * data member before calling this function.
     * @return Error status of setting up the sensor. EOK if successful.
     */
    errno_t (*open)(SensorLocation *loc, SensorContext *context);
    /**
     * Function responsible for reading the data associated with tag from the sensor.
     * @param loc The location of the sensor on the I2C bus.
     * @param tag The tag of the data type that should be read.
     * @param context The sensor context (context member).
     * @param buf A pointer to the byte array to store the data. Should be large enough to fit the max_return_size
     * bytes.
     * @param nbytes The number of bytes that were written into the byte array buffer.
     * @return Error status of reading from the sensor. EOK if successful.
     */
    errno_t (*read)(SensorLocation *loc, SensorTag tag, SensorContext *context, uint8_t *buf, uint8_t *nbytes);
} Sensor;

#endif // _SENSOR_API_H
