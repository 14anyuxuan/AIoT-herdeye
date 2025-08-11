#include "stepperMotor.h"


// ȫ����״̬����
static DoorState current_door_state = DOOR_CLOSED;


// ���õ������״̬
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



// ���Ƶ����ת������ULN2003������
void control_motor(uint8_t direction, uint16_t steps, uint32_t delay_ms_val) {
    // ���ѡ��ֹͣ״̬��ֱ�ӷ���  
    if (direction == STOP) {  
        return;  
    }  

    // ��鵱ǰ״̬����������ƶ���ִ��
    if (current_door_state == DOOR_MOVING) {
        return;
    }

    // ����״̬Ϊ�˶���
    current_door_state = DOOR_MOVING;
    
    // ������Ĳ������У��ʺ�ULN2003����28BYJ-48���
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
    static uint8_t seq_index = 0;  // ��̬�������ֵ�ǰλ�ã�����ÿ�δ�ͷ��ʼ
    
    for (uint16_t i = 0; i < steps; i++) {
        set_motor_pins(
            seq[seq_index][0], 
            seq[seq_index][1],
            seq[seq_index][2],
            seq[seq_index][3]
        );
        
        // ��ʱ
        delay_ms(delay_ms_val);
        
        // ������������ (��������)
        seq_index = (seq_index + step_direction + 8) % 8;
    }
    
    // ������ɺ�ر�������������������������
    set_motor_pins(0, 0, 0, 0);
    
    // ����״̬������ʵ�ʷ�����£�
    if (direction == CLOCKWISE) {
        current_door_state = DOOR_CLOSED;
    } else {
        current_door_state = DOOR_OPEN;
    }
}

// ���ź���
void open_door(void) {
    if (current_door_state == DOOR_OPEN || current_door_state == DOOR_MOVING) {
        return;
    }
    
    // ��ʱ�뿪�� (����ʵ�ʽ��ߵ�������)
    control_motor(COUNTERCLOCKWISE, DOOR_OPEN_STEPS, STEP_DELAY);
}

// ���ź���
void close_door(void) {
    if (current_door_state == DOOR_CLOSED || current_door_state == DOOR_MOVING) {
        return;
    }
    
    // ˳ʱ����� (����ʵ�ʽ��ߵ�������)
    control_motor(CLOCKWISE, DOOR_CLOSE_STEPS, STEP_DELAY);
}

// ��ȡ��ǰ��״̬
DoorState get_door_state(void) {
    return current_door_state;
}
