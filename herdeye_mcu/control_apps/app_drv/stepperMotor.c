#include "stepperMotor.h"


// 全局门状态变量
static DoorState current_door_state = DOOR_CLOSED;


// 设置电机引脚状态
void set_motor_pins(uint8_t a_plus, uint8_t a_minus, uint8_t b_plus, uint8_t b_minus) {
    if (a_plus) {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_IN1, 1);
    } else {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_IN1, 0);
    }

    if (a_minus) {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_IN2, 1);
    } else {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_IN2, 0);
    }

    if (b_plus) {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_IN3, 1);
    } else {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_IN3, 0);
    }

    if (b_minus) {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_IN4, 1);
    } else {
		GPIO_WriteBit(MOTOR_GPIO_PORT, MOTOR_IN4, 0);
    }
}



// 控制电机旋转，适配ULN2003驱动板
void control_motor(uint8_t direction, uint16_t steps, uint32_t delay_ms_val) {
    // 如果选择停止状态，直接返回  
    if (direction == STOP) {  
        return;  
    }  

    // 检查当前状态，如果正在移动则不执行
    if (current_door_state == DOOR_MOVING) {
        return;
    }

    // 更新状态为运动中
    current_door_state = DOOR_MOVING;
    
    // 四相八拍步进序列，适合ULN2003驱动28BYJ-48电机
    static const uint8_t seq[8][4] = {
        {1, 0, 0, 0},  // IN1
        {1, 1, 0, 0},  // IN1, IN2
        {0, 1, 0, 0},  // IN2
        {0, 1, 1, 0},  // IN2, IN3
        {0, 0, 1, 0},  // IN3
        {0, 0, 1, 1},  // IN3, IN4
        {0, 0, 0, 1},  // IN4
        {1, 0, 0, 1}   // IN4, IN1
    };

    int8_t step_direction = (direction == CLOCKWISE) ? 1 : -1;
    static uint8_t seq_index = 0;  // 静态变量保持当前位置，避免每次从头开始
    
    for (uint16_t i = 0; i < steps; i++) {
        set_motor_pins(
            seq[seq_index][0], 
            seq[seq_index][1],
            seq[seq_index][2],
            seq[seq_index][3]
        );
        
        // 延时
        delay_ms(delay_ms_val);
        
        // 更新序列索引 (带方向处理)
        seq_index = (seq_index + step_direction + 8) % 8;
    }
    
    // 操作完成后关闭所有输出，保护电机和驱动板
    set_motor_pins(0, 0, 0, 0);
    
    // 更新状态（根据实际方向更新）
    if (direction == CLOCKWISE) {
        current_door_state = DOOR_CLOSED;
    } else {
        current_door_state = DOOR_OPEN;
    }
}

// 开门函数
void open_door(void) {
    if (current_door_state == DOOR_OPEN || current_door_state == DOOR_MOVING) {
        return;
    }
    
    // 逆时针开门 (根据实际接线调整方向)
    control_motor(COUNTERCLOCKWISE, DOOR_OPEN_STEPS, STEP_DELAY);
}

// 关门函数
void close_door(void) {
    if (current_door_state == DOOR_CLOSED || current_door_state == DOOR_MOVING) {
        return;
    }
    
    // 顺时针关门 (根据实际接线调整方向)
    control_motor(CLOCKWISE, DOOR_CLOSE_STEPS, STEP_DELAY);
}

// 获取当前门状态
DoorState get_door_state(void) {
    return current_door_state;
}
