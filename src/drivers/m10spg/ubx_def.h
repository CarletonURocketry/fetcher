/**
 * @file ubx_def.c
 * @brief Definitions (structures and types) for the UBX message protocol
 *
 * Contains the building blocks of UBX messages, such as the message structure, data types, and
 */

#ifndef _UBX_DEF_
#define _UBX_DEF_

#include <stdint.h>

/** UBX header for all UBX protocol messages sent to the reciever */
typedef struct {
    uint8_t class;   /**< The class of this message, representing a category of messages, like debug or configuration */
    uint8_t id;      /**< The id of this message, representing the specific type of message in its class */
    uint16_t length; /**< The length of the message, including only the payload */
} UBXHeader;

/** UBX protcol style message, can be sent directly to the reciever */
typedef struct {
    UBXHeader header;   /**< A UBX protocol header*/
    void *payload;      /**< The payload of the message (length is stored in the header) */
    uint8_t checksum_a; /**< The first checksum byte of the message, including all fields past the synch characters */
    uint8_t checksum_b; /**< The second checksum byte */
} UBXFrame;

/** A struct representing the configuration layer selected in a configuration message (valset or valget) */
typedef enum {
    RAM_LAYER = 0x01,   /**< The current configuration - cleared if the reciever enters power saving mode */
    BBR_LAYER = 0x02,   /**< The battery backed memory configuration - not cleared unless the backup battery removed */
    FLASH_LAYER = 0x04, /**< The flash configuration - does not exist on the M10 MAX */
} UBXConfigLayer;

/** An enum representing the different sizes of values that a configuration message can contain */
typedef enum {
    UBX_TYPE_L = 1,  /**< One bit, occupies one byte */
    UBX_TYPE_U1 = 1, /**< One byte */
    UBX_TYPE_U2 = 2, /**< Two bytes, little endian */
    UBX_TYPE_U4 = 4, /**< Four bytes, little endian (excluding U8 because it's not used) */
} UBXValueType;

/** A struct representing the UBX-NAV-TIMEUTC (UTC Time) payload */
typedef struct {
    uint32_t iTOW; /**< The GPS time of week of the navigation epoch that created this payload */
    uint32_t tAcc; /**< A time accuracy measurement for the UTC time, in nanoseconds */
    int32_t nano;  /**< A time correction for the date that follows in this payload, in nanoseconds */
    uint16_t year; /**< A year from 1999 to 2099 (this can be incorrect if the chip was manufactured 20+ years ago) */
    uint8_t month; /**< A month from 1 to 12 */
    uint8_t day;   /**< Day of the month in the range 1 to 31 */
    uint8_t hour;  /**< Hour of the day in the range 0 to 23 */
    uint8_t min;   /**< Minute of the hour in the range 0 to 59 */
    uint8_t sec;   /**< Second of the minute in the range 0 to 60 */
    uint8_t flags; /**< Flags that describe if this time information is valid (see the interface description) */
} UBXUTCPayload;

/** Max bytes to be used for valset payload items (limit of 64 items per message) */
#define MAX_VALSET_ITEM_BYTES 128

/** A struct representing the UBX-VALSET (set configuration) payload */
typedef struct {
    uint8_t version;     /** The version of the message (always 0) */
    uint8_t layer;       /** The layer of this config, one of the UBXConfigLayer (typed to ensure one byte) */
    uint8_t reserved[2]; /** Reserved bytes */
    uint8_t config_items[MAX_VALSET_ITEM_BYTES]; /** An array of keys and value pairs */
} UBXValsetPayload;

/** A configuration key for enabling or disabling output of NMEA messages on I2C */
#define NMEA_I2C_OUTPUT_CONFIG_KEY 0x10720002

/** A configuration key for enabling or disabling input of poll requests for NMEA messages on I2C */
#define NMEA_I2C_INPUT_CONFIG_KEY 0x10710002

/** A configuration key for selecting the platform model of the reciever */
#define DYNMODEL_CONFIG_KEY 0x20110021

/** A configuration key for enabling or disabling the BeiDou satellites */
#define BSD_SIGNAL_CONFIG_KEY 0x10310022

/** A configuration key for selecting the number of milliseconds between measurements */
#define MEASUREMENT_RATE_CONFIG_KEY 0x30210001

/** A struct representing the UBX-NAV-STAT (navigation status) payload */
typedef struct {
    uint32_t iTOW;   /**< The GPS time of week of the navigation epoch that created this payload */
    uint8_t gpsFix;  /**< The type of fix */
    uint8_t flags;   /**< Navigation status flags */
    uint8_t fixStat; /**< The fix status */
    uint8_t flags2;  /**< More flags about navigation output */
    uint32_t ttff;   /**< The time to first fix, in milliseconds */
    uint32_t msss;   /**< Milliseconds since startup */
} UBXNavStatusPayload;

/** A struct representing the different fix types the GPS can have */
typedef enum {
    GPS_NO_FIX = 0x00,             /**< The gps has no fix, do not use data */
    GPS_DEAD_RECKONING = 0x01,     /**< Dead reckoning only (uses previous velocity and position information) */
    GPS_2D_FIX = 0x02,             /**< Two dimensional fix (no altitude) */
    GPS_3D_FIX = 0x03,             /**< Three dimensional fix */
    GPS_FIX_DEAD_RECKONING = 0x04, /**< Dead reckoning and gps combined */
    GPS_TIME_ONLY = 0x05,          /**< Time solution only, do not use other data */
} GPSFixType;

/** A struct representing the UBX-NAV-POSLLH (position and height) payload */
typedef struct {
    uint32_t iTOW;  /**< The GPS time of week of the navigation epoch that created this payload */
    int32_t lon;    /**< Longitude, in 0.0000001 * degrees */
    int32_t lat;    /**< Latitude, in 0.0000001 * degrees */
    int32_t height; /**< Height above ellipsoid in millimeters */
    int32_t hMSL;   /**< Height above mean sea level in millimeters */
    uint32_t hAcc;  /**< Horizontal accuracy measurement in millimeters */
    uint32_t vAcc;  /**< Vertical accuracy measurement in millimeters */
} UBXNavPositionPayload;

/** A conversion constant to go from the scale of UBX latitude (1E-7deg) to regular degrees */
#define LAT_SCALE_TO_DEGREES 1e7f

/** A conversion constant to go from the scale of UBX longitude (1E-7deg) to regular degrees */
#define LON_SCALE_TO_DEGREES 1e7f

/** A conversion constant to go from the scale of UBX altitude (mm) to meters */
#define ALT_SCALE_TO_METERS 1e3f

/** A struct representing the UBX-NAV-VELNED (velocity) payload */
typedef struct {
    uint32_t iTOW;   /**< The GPS time of week of the navigation epoch that created this payload */
    int32_t velN;    /**< North velocity component, in cm/s */
    int32_t velE;    /**< East velocity component, in cm/s */
    int32_t velD;    /**< Down velocity component, in cm/s */
    uint32_t speed;  /**< Speed (3-D), in cm/s */
    uint32_t gSpeed; /**< Ground speed (2-D), in cm/s */
    int32_t heading; /**< Heading of motion (2-D), in 0.00001 * degrees */
    uint32_t sAcc;   /**< Speed accuracy estimate, in cm/s */
    uint32_t cAcc;   /**< Course/heading accuracy estimate, in 0.00001 * degrees */
} UBXNavVelocityPayload;

/** A struct representing the UBX-ACK-ACK/UBX-ACK-NACK (acknowledgement) payload */
typedef struct {
    uint8_t clsId; /**< The class ID of the acknowledged or not acknowledged message */
    uint8_t msgId; /**< The message ID of the acknowledged or not acknowledged message */
} UBXAckPayload;

#endif // _UBX_DEF_
