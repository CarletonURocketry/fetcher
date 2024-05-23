#ifndef _PAC195X_H_
#define _PAC195X_H_

#include "../sensor_api.h"
#include <stdint.h>

/** The manufacturer ID for any PAC195X chip. */
#define MANU_ID 0x54

/** The product ID for the PAC1951-1 chip. */
#define PAC1951_1_PRODID 0x78
/** The product ID for the PAC1952-1 chip. */
#define PAC1952_1_PRODID 0x79
/** The product ID for the PAC1953-1 chip. */
#define PAC1953_1_PRODID 0x7A
/** The product ID for the PAC1954-1 chip. */
#define PAC1954_1_PRODID 0x7B
/** The product ID for the PAC1951-2 chip. */
#define PAC1951_2_PRODID 0x7C
/** The product ID for the PAC1952-2 chip. */
#define PAC1952_2_PRODID 0x7D

/** Revision ID of the initial release. */
#define PAC195X_INIT_REL 0x02

typedef enum {
    SAMPLE_1024_SPS_AD = 0x00,   /**< 1024 SPS adaptive accumulation (default). */
    SAMPLE_256_SPS_AD = 0x10,    /**< 256 SPS adaptive accumulation. */
    SAMPLE_64_SPS_AD = 0x20,     /**< 64 SPS adaptive accumulation. */
    SAMPLE_8_SPS_AD = 0x30,      /**< 8 SPS adaptive accumulation. */
    SAMPLE_1024_SPS = 0x40,      /**< 1024 SPS. */
    SAMPLE_256_SPS = 0x50,       /**< 256 SPS. */
    SAMPLE_64_SPS = 0x60,        /**< 64 SPS. */
    SAMPLE_8_SPS = 0x70,         /**< 8 SPS. */
    SAMPLE_SINGLE_SHOT = 0x80,   /**< Single shot mode. */
    SAMPLE_SINGLE_SHOT8X = 0x90, /**< Single shot 8X. */
    SAMPLE_FAST = 0xA0,          /**< Fast mode. */
    SAMPLE_BURST = 0xB0,         /**< Burst mode. */
    SAMPLE_SLEEP = 0xF0,         /**< Sleep. */
} pac195x_sm_e;

int pac195x_get_manu_id(SensorLocation const *loc, uint8_t *id);
int pac195x_get_prod_id(SensorLocation const *loc, uint8_t *id);
int pac195x_get_rev_id(SensorLocation const *loc, uint8_t *id);

int pac195x_set_sample_mode(SensorLocation const *loc, pac195x_sm_e mode);

int pac195x_refresh(SensorLocation const *loc);
int pac195x_refresh_v(SensorLocation const *loc);
int pac195x_refresh_g(SensorLocation const *loc);

#endif // _PAC195X_H_
