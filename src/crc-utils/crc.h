/**
 * @file crc.h
 * @brief Header file containing function prototypes for calculating cyclic redundancy checks
 */
#ifndef _CRC_H_
#define _CRC_H_

#include <stddef.h>
#include <stdint.h>

/** Structure that contains information required for a CRC-8 lookup table */
typedef struct crc8_lookup_t {
    uint8_t table[256];
} CRC8LookupTable;

uint8_t calculate_crc8(const uint8_t *buf, size_t nbytes, const CRC8LookupTable *lookup, uint8_t initial);
void generate_crc8_lookup(CRC8LookupTable *lookup, uint8_t polynomial);
uint8_t calculate_crc8_bitwise(const uint8_t *buf, size_t nbytes, uint8_t polynomial, uint8_t initial);

#endif // _CRC_H_