#ifndef __RELAY_H__
#define __RELAY_H__

#include "board_init.h"

#define DEVICE_LIGHT  1  // PA5
#define DEVICE_PUMP   2  // PA6
#define DEVICE_MOTOR  3  // PA7

void relay_init();
void set_realy_data(uint8_t number, uint8_t value);

#endif
