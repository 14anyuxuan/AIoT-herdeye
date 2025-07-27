#include "relay.h"


void relay_init(){
	GPIO_WriteBit(GPIOA, GPIO_PIN_2, 0);
	GPIO_WriteBit(GPIOA, GPIO_PIN_3, 0);
	GPIO_WriteBit(GPIOA, GPIO_PIN_5, 0);
}


void set_realy_data(uint8_t number, uint8_t value){
	switch (number){
		case DEVICE_LIGHT:
			GPIO_WriteBit(GPIOA, GPIO_PIN_5, value);
			break;
		case DEVICE_PUMP:
			GPIO_WriteBit(GPIOA, GPIO_PIN_6, value);
			break;
		case DEVICE_MOTOR:
			GPIO_WriteBit(GPIOA, GPIO_PIN_7, value);
			break;
	}
}


