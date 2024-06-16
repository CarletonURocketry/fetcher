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
    UBX_MSG_NONE = 0x00,     /**< No message (not a real message type) */
    UBX_MSG_ANY = 0x01,      /**< Any message (not a real message type) */
    UBX_MSG_ACK = 0x02,      /**< Acknowledgment message */
    UBX_MSG_NACK = 0x03,     /**< Non-acknowledgement message */
    UBX_MSG_ACK_NACK = 0x04, /**< Either the acknowledgement or non-acknowledgement message (not a real message type) */
    UBX_MSG_NAV_UTC = 0x05,  /**< UTC time information, recieved formatted as a human readable date */
    UBX_MSG_NAV_POSLLH = 0x06, /**< Latitude and longitude information, along with altitude */
    UBX_MSG_NAV_VELNED = 0x07, /**< Velocity and heading information */
    UBX_MSG_NAV_STAT = 0x08,   /**< GPS status information about fix, fix type */
    UBX_MSG_MON_VER = 0x09,    /**< Firmware version information */
    UBX_MSG_NAV_PVT = 0x0a,    /**< Position, velocity, time information (reccomended) */
    UBX_MSG_RST = 0x0b,        /**< Reciever reset message */
} M10SPGMessageType;

/**
 * A pointer to a function that will recieve a buffer containing a populated message containing a complete payload and
 * the type of the message that was recieved. The handler is reponsible for using the data, after which it will be
 * discarded
 *
 * @param msg The message that was last read from the gps and which now needs to be consumed
 * @return 0 if the message was handled successfully, -1 if there was an error processing the message
 */
typedef int (*M10SPGMessageHandler)(UBXFrame *msg);

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
int m10spg_read(M10SPGContext *ctx, M10SPGMessageType msg_type, UBXFrame *recv, size_t size);
int m10spg_register_periodic(M10SPGContext *ctx, M10SPGMessageHandler handler, M10SPGMessageType msg_type);
int m10spg_is_type(UBXFrame *msg, M10SPGMessageType type);
void m10spg_sleep_epoch();

#endif // _MAXM10S_
