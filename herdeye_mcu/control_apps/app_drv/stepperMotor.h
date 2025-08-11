#ifndef __STEPPERMOTOR_H__
#define __STEPPERMOTOR_H__

#include "board_init.h"


// 定义电机控制引脚
#define MOTOR_IN1  GPIO_PIN_13  // 连接到ULN2003的IN1
#define MOTOR_IN2  GPIO_PIN_14  // 连接到ULN2003的IN2
#define MOTOR_IN3  GPIO_PIN_11  // 连接到ULN2003的IN3
#define MOTOR_IN4  GPIO_PIN_12  // 连接到ULN2003的IN4

#define MOTOR_GPIO_PORT GPIOD


// 定义电机旋转方向
#define CLOCKWISE 1  
#define COUNTERCLOCKWISE 2  
#define STOP 0  

#define DOOR_OPEN_STEPS  1024    // ULN2003通常驱动28BYJ-48电机，需要更多步数
#define DOOR_CLOSE_STEPS 1024    
#define STEP_DELAY       1       // 步进延迟(ms)，28BYJ-48适合较低速度

// 门状态枚举
typedef enum {
    DOOR_OPEN,
    DOOR_CLOSED,
    DOOR_MOVING,  
    DOOR_ERROR    
} DoorState;


// 函数声明
void open_door(void);
void close_door(void);
DoorState get_door_state(void);
void control_motor(uint8_t direction, uint16_t steps, uint32_t delay);


#endif
