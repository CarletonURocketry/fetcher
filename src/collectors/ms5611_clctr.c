#include "../drivers/ms5611/ms5611.h"
#include "collectors.h"
#include "sensor_api.h"

void *ms5611_collector(void *args) {

    // Create MS5611 instance
    Sensor ms5611;
    ms5611_init(&ms5611, clctr_args(args)->bus, clctr_args(args)->addr, PRECISION_HIGH);
    uint8_t ms5611_context[sensor_get_ctx_size(ms5611)];
    sensor_set_ctx(&ms5611, ms5611_context);
    errno_t err = sensor_open(ms5611);
    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return (void *)err;
    }

    float data;
    size_t nbytes;
    for (;;) {

        // Print pressure
        sensor_read(ms5611, TAG_PRESSURE, &data, &nbytes);
        sensor_write_data(stdout, TAG_PRESSURE, &data);

        // Print temperature
        sensor_read(ms5611, TAG_TEMPERATURE, &data, &nbytes);
        sensor_write_data(stdout, TAG_TEMPERATURE, &data);
    }
}
