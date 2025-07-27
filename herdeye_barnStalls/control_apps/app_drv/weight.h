#ifndef __WEIGHT_H__
#define __WEIGHT_H__

#include "board_init.h"

#define HX711_SCK_H				GPIO_WriteBit(GPIOE, GPIO_PIN_11, 1)
#define HX711_SCK_L				GPIO_WriteBit(GPIOE, GPIO_PIN_11, 0)
 
#define HX711_DT				GPIO_ReadInDataBit(GPIOE, GPIO_PIN_12)

unsigned long HX711_GetData(void);
float get_current_weight(void);
void weight_reset_init();

#endif
