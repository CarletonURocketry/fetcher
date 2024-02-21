/**
 * @file crc.c
 * @brief This file contains the required functions and data to calculate cyclic redundancy checks
 */
#include "crc.h"

/**
 * Calculates an 8 bit cyclic redundancy check (CRC-8) for the provided data using a lookup table
 * @param buff A pointer to the data to have its CRC calculated
 * @param n_bytes The length of the data in bytes
 * @param lookup A lookup table where an index stores its own CRC-8, must be 256 bytes if provided. Ignored if null
 * @param initial The initial value of the CRC, set to 0x00 as default
 * @return uint8_t The calculated CRC
 */
uint8_t calculate_crc8(const uint8_t *buf, size_t nbytes, const CRC8LookupTable *lookup, uint8_t initial) {
    uint8_t crc = initial;
    for (size_t byte = 0; byte < nbytes; byte++) {
        crc = lookup->table[crc ^ buf[byte]];
    }
    return crc;
}
/**
 * Calculates an 8 bit cyclic redundancy check (CRC-8) for the provided data without a lookup
 * @param buff A pointer to the data to have its CRC calculated
 * @param n_bytes The length of the data in bytes
 * @param polynomial The 8 bit polynomial used to calculate the CRC
 * @param initial The initial value of the CRC, set to 0x00 as default
 * @return uint8_t The calculated CRC
 */
uint8_t calculate_crc8_bitwise(const uint8_t *buf, size_t nbytes, uint8_t polynomial, uint8_t initial) {
    uint8_t crc = initial;
    for (size_t byte = 0; byte < nbytes; byte++) {
        crc ^= buf[byte];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                // Discard the highest bit (implicit XOR), then divide by the polynomial
                crc <<= 1;
                crc ^= polynomial;
            } else {
                // Continue until the highest bit is set
                crc <<= 1;
            }
        }
    }
    return crc;
}
/**
 * Generates a lookup table for an 8 bit cyclic redundancy check (CRC-8)
 * @param lookup The lookup table to store the calculated CRC values in
 * @param polynomial The 8 bit polynomial to generate the lookup table with, has an implicit 1 appended after the MSB
 */
void generate_crc8_lookup(CRC8LookupTable *lookup, uint8_t polynomial) {
    for (uint8_t curr = 0; curr < 0xFF; curr++) {
        uint8_t remainder = curr;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (remainder & 0x80) {
                // Discard the highest bit (implicit XOR), then divide by the polynomial
                remainder <<= 1;
                remainder ^= polynomial;
            } else {
                // Continue until the highest bit is set
                remainder <<= 1;
            }
        }
        lookup->table[curr] = remainder;
    }
}