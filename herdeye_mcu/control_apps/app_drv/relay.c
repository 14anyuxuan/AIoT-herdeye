#include "relay.h"


void relay_init(){
	GPIO_WriteBit(GPIOA, GPIO_PIN_5, 0);
	GPIO_WriteBit(GPIOA, GPIO_PIN_6, 0);
	GPIO_WriteBit(GPIOA, GPIO_PIN_7, 0);
	GPIO_WriteBit(GPIOB, GPIO_PIN_11, 0);
	GPIO_WriteBit(GPIOB, GPIO_PIN_12, 0);
}


void set_realy_data(uint8_t number, uint8_t value){
	
	switch (number){
		case DEVICE_LIGHT:
			GPIO_WriteBit(GPIOA, GPIO_PIN_5, value);
			break;
		case DEVICE_PUMP:
			GPIO_WriteBit(GPIOA, GPIO_PIN_6, value);
			break;
		case DEVICE_MOTOR_CUT:
			GPIO_WriteBit(GPIOA, GPIO_PIN_7, value);
			break;
		case DEVICE_MOTOR_FAN:
			GPIO_WriteBit(GPIOB, GPIO_PIN_12, value);
			break;
		case DEVICE_HEAT:
			GPIO_WriteBit(GPIOB, GPIO_PIN_11, !value);
			break;
	}
}
	