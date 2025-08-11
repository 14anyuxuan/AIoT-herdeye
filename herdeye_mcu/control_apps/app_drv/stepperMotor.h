#ifndef __STEPPERMOTOR_H__
#define __STEPPERMOTOR_H__

#include "board_init.h"


// ��������������
#define MOTOR_IN1  GPIO_PIN_13  // ���ӵ�ULN2003��IN1
#define MOTOR_IN2  GPIO_PIN_14  // ���ӵ�ULN2003��IN2
#define MOTOR_IN3  GPIO_PIN_11  // ���ӵ�ULN2003��IN3
#define MOTOR_IN4  GPIO_PIN_12  // ���ӵ�ULN2003��IN4

#define MOTOR_GPIO_PORT GPIOD


// ��������ת����
#define CLOCKWISE 1  
#define COUNTERCLOCKWISE 2  
#define STOP 0  

#define DOOR_OPEN_STEPS  1024    // ULN2003ͨ������28BYJ-48�������Ҫ���ಽ��
#define DOOR_CLOSE_STEPS 1024    
#define STEP_DELAY       1       // �����ӳ�(ms)��28BYJ-48�ʺϽϵ��ٶ�

// ��״̬ö��
typedef enum {
    DOOR_OPEN,
    DOOR_CLOSED,
    DOOR_MOVING,  
    DOOR_ERROR    
} DoorState;


// ��������
void open_door(void);
void close_door(void);
DoorState get_door_state(void);
void control_motor(uint8_t direction, uint16_t steps, uint32_t delay);


#endif
