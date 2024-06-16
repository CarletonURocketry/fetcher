/**
 * @file m10spg.h
 * @brief Header file containing the necessary types and function prototypes for controlling the MAX-M10S gps sensor.
 *
 * Header file containing the necessary types and function prototypes for controlling the MAX-M10S gps sensor.
 */

#ifndef _MAXM10S_
#define _MAXM10S_

#include "../sensor_api.h"
#include "ubx_def.h"
#include <stdint.h>

/** An enum representing the commands that can be used for reading from the M10SPG, where the value of the enum
 * represents the class and id, in the high and low bytes of a uint16, respectively */
typedef enum {
    UBX_MSG_NONE = 0X00, /**< No message */
    UBX_MSG_ANY,         /**< Any message */
    UBX_MSG_ACK,         /**< Acknowledgment message */
    UBX_MSG_NACK,        /**< Non-acknowledgement message */
    UBX_MSG_ACK_NACK,    /**< Acknowledgement or non-acknowledgement message */
    UBX_MSG_NAV_UTC,     /**< UTC time information, recieved formatted as a human readable date */
    UBX_MSG_NAV_POSLLH,  /**< Latitude and longitude information, along with altitude */
    UBX_MSG_NAV_VELNED,  /**< Velocity and heading information */
    UBX_MSG_NAV_STAT,    /**< GPS status information about fix, fix type */
    UBX_MSG_MON_VER,     /**< Firmware version information */
    UBX_MSG_NAV_PVT,     /**< Position, velocity, time information (reccomended) */
} M10SPGMessageType;

/**
 * A pointer to a function that will recieve a buffer containing a populated message containing a complete payload and
 * the type of the message that was recieved. The handler is reponsible for using the data, after which it will be
 * discarded
 *
 * @param msg The message that was last read from the gps and which now needs to be consumed
 * @param type The type of msg (containing the same information as the class and id fields)
 * @return 0 if the message was handled successfully, -1 if there was an error processing the message
 */
typedef int (*M10SPGMessageHandler)(UBXFrame *msg, M10SPGMessageType type);

#define MAX_PERIODIC_MESSAGES 1

/** A struct that describes a M10SPG sensor */
typedef struct {
    const SensorLocation *loc; /**< The location of this sensor on the I2C bus */
    struct {
        M10SPGMessageType type;       /**< The type of message that this handler takes */
        M10SPGMessageHandler handler; /**< The function to call when finding a message of the specified type */
    } handlers[MAX_PERIODIC_MESSAGES];
} M10SPGContext;

int m10spg_open(M10SPGContext *ctx, SensorLocation *loc);
int m10spg_read(M10SPGContext *ctx, M10SPGMessageType msg_type, uint8_t *buf, size_t size);
int m10spg_register_periodic(M10SPGContext *ctx, M10SPGMessageHandler handler, M10SPGMessageType msg_type);
M10SPGMessageType m10spg_get_payload_type(UBXFrame *msg);
void m10spg_sleep_epoch(M10SPGContext *ctx);

#endif // _MAXM10S_
