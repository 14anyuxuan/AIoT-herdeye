#include "stepperMotor.h"


// 设置电机引脚状态
void set_motor_pins(uint8_t a_plus, uint8_t a_minus, uint8_t b_plus, uint8_t b_minus) {
    if (a_plus) {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_A_PLUS, 1);
    } else {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_A_PLUS, 0);
    }

    if (a_minus) {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_A_MINUS, 1);
    } else {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_A_MINUS, 0);
    }

    if (b_plus) {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_B_PLUS, 1);
    } else {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_B_PLUS, 0);
    }

    if (b_minus) {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_B_MINUS, 1);
    } else {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_B_MINUS, 0);
    }
}

// 控制电机旋转  
void control_motor(uint8_t direction, uint16_t steps, uint32_t delay) {  
    // 如果选择停止状态，直接返回  
    if (direction == STOP) {  
        return;  
    }  

    for (uint16_t i = 0; i < steps; i++) {  
        if (direction == CLOCKWISE) {  
            // 顺时针旋转序列  
            set_motor_pins(1, 0, 1, 0);  // A+, B+  
            app_delay_ms(delay);  
            set_motor_pins(1, 0, 0, 1);  // A+, B-  
            app_delay_ms(delay);  
            set_motor_pins(0, 1, 0, 1);  // A-, B-  
            app_delay_ms(delay);  
            set_motor_pins(0, 1, 1, 0);  // A-, B+  
            app_delay_ms(delay);  
        } else if (direction == COUNTERCLOCKWISE) {  
            // 逆时针旋转序列  
            set_motor_pins(0, 1, 1, 0);  // A-, B+  
            app_delay_ms(delay);  
            set_motor_pins(0, 1, 0, 1);  // A-, B-  
            app_delay_ms(delay);  
            set_motor_pins(1, 0, 0, 1);  // A+, B-  
            app_delay_ms(delay);  
            set_motor_pins(1, 0, 1, 0);  // A+, B+  
            app_delay_ms(delay);  
        }  
    }  
}  




