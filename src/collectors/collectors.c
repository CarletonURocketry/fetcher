#include "collectors.h"
#include <string.h>

static const clctr_entry_t COLLECTORS[] = {
    {.name = "SHT41", .collector = sht41_collector},
    {.name = "SYSCLOCK", .collector = sysclock_collector},
    {.name = "MS5611", .collector = ms5611_collector},
    {.name = "LSM6DSO32", .collector = lsm6dso32_collector},
};

/**
 * Searches for a collector matching the sensor name in the list of implemented collectors.
 * WARNING: Currently case sensitive.
 * @param sensor_name The name of the sensor to find a collector thread for.
 * @return A function pointer to the collector thread, or NULL if no match is found.
 */
collector_t collector_search(const char *sensor_name) {
    for (uint8_t i = 0; i < sizeof(COLLECTORS) / sizeof(clctr_entry_t); i++) {
        if (!strcmp(sensor_name, COLLECTORS[i].name)) {
            return COLLECTORS[i].collector;
        }
    }
    return NULL;
}
