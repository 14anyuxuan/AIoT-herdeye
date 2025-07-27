#ifndef __STEPPERMOTOR_H__
#define __STEPPERMOTOR_H__

#include "board_init.h"
#include "ws2812.h"


// 定义电机控制引脚
#define MOTOR_A_PLUS  GPIO_PIN_14  // E14
#define MOTOR_A_MINUS GPIO_PIN_15  // E15
#define MOTOR_B_PLUS  GPIO_PIN_12  // E12
#define MOTOR_B_MINUS GPIO_PIN_13  // E13

#define MOTOR_GPIO_PORT GPIOE

// 定义电机旋转方向
#define CLOCKWISE 1  
#define COUNTERCLOCKWISE 2  
#define STOP 0  

void control_motor(uint8_t direction, uint16_t steps, uint32_t delay);


#endif
