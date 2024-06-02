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
    UBX_NAV_UTC,    /**< UTC time information, recieved formatted as a human readable date */
    UBX_NAV_POSLLH, /**< Latitude and longitude information, along with altitude */
    UBX_NAV_VELNED, /**< Velocity and heading information */
    UBX_NAV_STAT,   /**< GPS status information about fix, fix type */
    UBX_MON_VER,    /**< Firmware version information */
} M10SPG_cmd_t;

int m10spg_open(const SensorLocation *loc);
int m10spg_send_command(const SensorLocation *loc, M10SPG_cmd_t command, void *response, size_t size);

#endif // _MAXM10S_
