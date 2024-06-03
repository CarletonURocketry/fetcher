#ifndef _BOARD_ID_H_
#define _BOARD_ID_H_

#include <stdint.h>

/** The address of the EEPROM containing the board ID for the sensor board. */
#define BOARD_ID_ADDR 0x50

const char *read_sensor_name(const char *board_id, char *sensor_name, uint8_t nbytes);
const char *read_sensor_addresses(const char *board_id, uint8_t *addresses, uint8_t *naddrs);

#endif // _BOARD_ID_H_
