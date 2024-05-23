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

int pac195x_get_manu_id(SensorLocation const *loc, uint8_t *id);
int pac195x_get_prod_id(SensorLocation const *loc, uint8_t *id);
int pac195x_get_rev_id(SensorLocation const *loc, uint8_t *id);

int pac195x_refresh(SensorLocation const *loc);
int pac195x_refresh_v(SensorLocation const *loc);
int pac195x_refresh_g(SensorLocation const *loc);

#endif // _PAC195X_H_
