#include "sensor_api.h"

/**
 * Utility function for copying memory in big-endian format.
 * @param dest The destination buffer for data copied from src.
 * @param src The source buffer for data to be copied into dest.
 * @param nbytes The number of bytes to copy from src into dest.
 */
void memcpy_be(void *dest, const void *src, const size_t nbytes){
    for (size_t i = nbytes; i > 0; i--){
        ((uint8_t*)dest)[i - 1] = ((const uint8_t*)src)[nbytes - i];
    }
}
