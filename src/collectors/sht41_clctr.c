#define SHT41_USE_CRC_LOOKUP
#include "../drivers/sht41/sht41.h"
#include "collectors.h"
#include "sensor_api.h"

void *sht41_collector(void *args) {

    // Create SHT41 instance
    Sensor sht41;
    sht41_init(&sht41, clctr_args(args)->bus, clctr_args(args)->addr, PRECISION_HIGH);

    uint8_t sht41_context[sensor_get_ctx_size(sht41)];

    sensor_set_ctx(&sht41, sht41_context);
    errno_t err = sensor_open(sht41);
    if (err != EOK) {
        fprintf(stderr, "%s\n", strerror(err));
        return (void *)err;
    }

    size_t nbytes;
    float data;
    for (;;) {
        // Read temperature
        sensor_read(sht41, TAG_TEMPERATURE, &data, &nbytes);
        sensor_write_data(stdout, TAG_TEMPERATURE, &data);

        // Read humidity
        sensor_read(sht41, TAG_HUMIDITY, &data, &nbytes);
        sensor_write_data(stdout, TAG_HUMIDITY, &data);
    }
}
