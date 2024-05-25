/**
 * @file m10spg.h
 * @brief Header file containing the necessary types and function prototypes for controlling the MAX-M10S gps sensor.
 *
 * Header file containing the necessary types and function prototypes for controlling the MAX-M10S gps sensor.
 */

#ifndef _MAXM10S_
#define _MAXM10S_

#include "../sensor_api.h"
#include <stdint.h>

/** An enum representing the commands that can be used for polling M10SPG data. */
typedef enum {
    UBX_NAV_UTC,
    UBX_NAV_POSLLH,
    UBX_NAV_VELNED,
    UBX_NAV_STAT,
    UBX_MON_VER,
} M10SPG_cmd_t;

int m10spg_open(const SensorLocation *loc);
int m10spg_send_command(const SensorLocation *loc, M10SPG_cmd_t command, void *response, size_t size);

#endif // _MAXM10S_
