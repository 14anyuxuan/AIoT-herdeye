#ifndef __BRIGHETNESS_H__
#define __BRIGHETNESS_H__

#include "board_init.h"


#define BRIGHETNESS_ADC_PORT               ADC1
#define BRIGHETNESS_ADC_CHN_NUM            17 		/* select the adc fixed channel 0*/
#define BRIGHETNESS_ADC_SLOT_MASK 				 0x01		/* enable fixed channel 0*/

void brighetness_init(void);
uint32_t brighetness_adc_run_conv(void);

uint8_t brighetness_data();


#endif