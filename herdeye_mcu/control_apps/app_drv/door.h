#ifndef __DOOR_H__
#define __DOOR_H__


#include "board_init.h"

#define DOOR_TIM_FREQ       CLOCK_SYS_FREQ

#define DOOR_TIM_PORT       (TIM_Type *)TIM3
#define DOOR_TIM_CHANNEL    TIM_CHN_1

void boor_init();
void set_boor_flag(uint8_t flag);

#endif
