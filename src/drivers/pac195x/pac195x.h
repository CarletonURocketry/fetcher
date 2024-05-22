#ifndef _PAC195X_H_
#define _PAC195X_H_

#include "../sensor_api.h"
#include <stdint.h>

int pac195x_get_manu_id(SensorLocation const *loc, uint8_t *id);

#endif // _PAC195X_H_
