#ifndef __RELAY_H__
#define __RELAY_H__

#include "board_init.h"

#define DEVICE_LIGHT  			1  // PA5
#define DEVICE_PUMP   			2  // PA6
#define DEVICE_MOTOR  			3  // PA7
#define DEVICE_MOTOR_FAN  		4  // PB11 == 1	PB12 == 0
#define DEVICE_MOTOR_CUT		5  // PB13 == 1 PB14 == 0
#define DEVICE_HEAT  			6  // PB10 == 0

void relay_init();
void set_realy_data(uint8_t number, uint8_t value);

#endif
