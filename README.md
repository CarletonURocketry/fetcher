# fetcher

A process for fetching data from sensors using the I2C protocol.

## Usage

See the help documentation with the command: `use fetcher`.

The output of fetcher looks like:

```console
Time: 1295 ms
Temperature: 25.02 C
Pressure: 99.94 kPa
...
```

Sensor data output can either be printed in plain-text, or it can be read from the named message queue
`fetcher/sensors`.

Messages on the message queue start with a one-byte type specifier which is one of the following:

```c
/** Describes possible data types that fetcher is capable of producing. */
typedef enum {
    TAG_TEMPERATURE = 0,      /**< Temperature in degrees Celsius */
    TAG_PRESSURE = 1,         /**< Pressure in kilo Pascals */
    TAG_HUMIDITY = 2,         /**< Humidity in % relative humidity */
    TAG_TIME = 3,             /**< Time in milliseconds */
    TAG_ALTITUDE_SEA = 4,     /**< Altitude above sea level in meters */
    TAG_ALTITUDE_REL = 5,     /**< Altitude above launch height in meters */
    TAG_ANGULAR_VEL = 6,      /**< Angular velocity in degrees per second */
    TAG_LINEAR_ACCEL_REL = 7, /**< Relative linear acceleration in meters per second squared */
    TAG_LINEAR_ACCEL_ABS = 8, /**< Absolute linear acceleration in meters per second squared */
} SensorTag;
```

Followed by data representing the measurement.

Temperature, pressure, humidity and altitude are floats.
Time is a 32 bit integer.
Linear acceleration and angular velocity are 3D vectors (`vec3d_t`) of 3 floats.

## Board ID EEPROM Encoding

In order for fetcher to recognize the sensors on the board, the EEPROM must encode the ID in this format:

```
CU InSpace <board name>
REV <x>: <manufacture date>
<sensor 1 id> <address>
<sensor 2 id> <address> <address2>
```

**Fields:**

- `<board name>`: The name of the board. Ex: `Sensor board`
- `REV <x>`: The revision of the board. Ex: `REV B`
- `<sensor id>`: The identifier of the sensor. Ex: `MS5611`
- `<address>`: The 7 bit I2C address of the sensor in hexadecimal (without the leading 0x). Ex: `77`
  - It is possible to have more than one address per sensor type for boards with duplicates

<!--- Links --->

[packager]: https://github.com/CarletonURocketry/packager
