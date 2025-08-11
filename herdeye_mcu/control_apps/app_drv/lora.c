#include "lora.h"
#include  "cJSON.h"

#define LORA_RX_BUF_SIZE 1024

volatile char lora_uart_rx_buf[LORA_RX_BUF_SIZE];
volatile uint16_t lora_rx_index = 0;
volatile bool lora_frame_ready = false;
volatile bool new_data_flag = false;
int SequenceNumber = 0;

int LightValue = 0;
int WaterPumpValue = 0;
int MotorValue = 0;
int DoorValue = 0;
int HeatValue = 0;
int FanValue = 0;

volatile bool effect_running = false; // ����Ч��ѭ���Ƿ�����
volatile uint8_t current_mode = 0xFF; // ��¼��ǰģʽ

void lora_init(){
	UART_Init_Type uart_init;

	// USART ��ʼ������
	uart_init.ClockFreqHz   = CLOCK_APB2_FREQ;
	uart_init.BaudRate 			= 9600u;										//���ڲ�����
	uart_init.WordLength    = UART_WordLength_8b;						//�ֳ�Ϊ8λ���ݸ�ʽ
	uart_init.StopBits      = UART_StopBits_1;							//һ��ֹͣλ
	uart_init.Parity        = UART_Parity_None;								//����żУ��λ
	uart_init.XferMode      = UART_XferMode_RxTx;				        //�շ�ģʽ
	uart_init.HwFlowControl = UART_HwFlowControl_None;                  //��Ӳ������������	
	uart_init.EnableSwapTxRxXferSignal = false;
	uart_init.XferSignal    = UART_XferSignal_Normal;
	
	
	UART_Init(UART2, &uart_init);										//��ʼ������2
	/* Enable RX interrupt. */
	UART_EnableInterrupts(UART2, UART_INT_RX_DONE, true);			
	NVIC_SetPriority(UART2_IRQn, 0);	
	NVIC_EnableIRQ(UART2_IRQn);
	/* Enable UART. */
	UART_Enable(UART2, true);
	
	
}

void lora_uart_putchar(uint8_t c)
{
    while ( 0u == (UART_STATUS_TX_EMPTY & UART_GetStatus(UART2)) )
    {}
    UART_PutData(UART2, (uint8_t)(c));
}


void lora_uart_putstr(uint8_t *str)
{
    while ((*str) != '\0')
    {
        lora_uart_putchar(*str);
        str++;
    }
}


void UART2_IRQHandler(void) {
    if (UART_GetInterruptStatus(UART2) & UART_INT_RX_DONE) {
        uint8_t data = UART_GetData(UART2);
        
        // ��ֹ������������һλ��������
        if (lora_rx_index < (LORA_RX_BUF_SIZE - 1)) {
            lora_uart_rx_buf[lora_rx_index++] = data;
            
            // ���֡��������������\n��β��
            if (data == '\n') {
                lora_uart_rx_buf[lora_rx_index] = '\0'; // ��ӽ�����
                lora_frame_ready = true; // ֪ͨ��ѭ��
            }
        }
        UART_ClearInterruptStatus(UART2, UART_INT_RX_DONE);
    }
}

void lora_ProcessData() {
    if (lora_frame_ready) {
        // �����ٽ�������ֹ�жϷ�ֹ�����޸�
        bool irq = __get_PRIMASK();
        __disable_irq();
        
        printf("Received: %s\r\n", lora_uart_rx_buf);
        
        // ����JSON����
        bool parse_success = Parse_JSON_Data((const char*)lora_uart_rx_buf);
        
        if (parse_success) {
            // ����ACK��Ӧ
            char ack_msg[32];
            snprintf(ack_msg, sizeof(ack_msg), "ACK%d\r\n", SequenceNumber);
            lora_uart_putstr((uint8_t *)ack_msg);
            
            // ִ�п���
            loraControl(LightValue, WaterPumpValue, MotorValue, DoorValue, HeatValue, FanValue);
            
            printf("Control applied: L:%d WP:%d M:%d D:%d\n", 
                   LightValue, WaterPumpValue, MotorValue, DoorValue);
        } else {
            printf("JSON parse failed, skipping control\n");
        }
        
        // ������ɺ�����״̬
        lora_rx_index = 0;
        lora_frame_ready = false;
        
        // �ָ��ж�״̬
        if (!irq) __enable_irq();
    }
}

bool Parse_JSON_Data(const char* json_str) {
    cJSON* root = cJSON_Parse(json_str);
    if (!root) {
        printf("JSON Parse Error: %s\n", cJSON_GetErrorPtr());
        return false;
    }
	
	bool parse_success = true;


    // ����LightValueֵ (ֻ����0��1)
    cJSON* Light_item = cJSON_GetObjectItem(root, "LightValue");
    if (cJSON_IsNumber(Light_item) && (Light_item->valueint == 0 || Light_item->valueint == 1)) {
        LightValue = Light_item->valueint;
        printf("LightValue set to: %d\n", LightValue);
    } else {
        printf("Invalid LightValue (must be 0 or 1)\n");
		parse_success = false;
    }
    
    // ����WaterPumpValueֵ (ֻ����0��1)
    cJSON* WaterPump_item = cJSON_GetObjectItem(root, "WaterPumpValue");
    if (cJSON_IsNumber(WaterPump_item) && (WaterPump_item->valueint == 0 || WaterPump_item->valueint == 1)) {
        WaterPumpValue = WaterPump_item->valueint;
        printf("WaterPumpValue set to: %d\n", WaterPumpValue);
    } else {
        printf("Invalid WaterPumpValue (must be 0 or 1)\n");
		parse_success = false;
    }

    // ����MotorValueֵ (ֻ����0��1)
    cJSON* Motor_item = cJSON_GetObjectItem(root, "MotorValue");
    if (cJSON_IsNumber(Motor_item) && (Motor_item->valueint == 0 || Motor_item->valueint == 1)) {
        MotorValue = Motor_item->valueint;
        printf("MotorValue set to: %d\n", MotorValue);
    } else {
        printf("Invalid MotorValue (must be 0 or 1)\n");
		parse_success = false;
    }

    // ����DoorValueֵ (ֻ����0��1)
    cJSON* Door_item = cJSON_GetObjectItem(root, "DoorValue");
    if (cJSON_IsNumber(Door_item) && (Door_item->valueint == 0 || Door_item->valueint == 1)) {
        DoorValue = Door_item->valueint;
        printf("DoorValue set to: %d\n", DoorValue);
    } else {
        printf("Invalid DoorValue (must be 0 or 1)\n");
		parse_success = false;
    }
	
	// ����HeatValueֵ (ֻ����0��1)
    cJSON* Heat_item = cJSON_GetObjectItem(root, "HeatValue");
    if (cJSON_IsNumber(Heat_item) && (Heat_item->valueint == 0 || Heat_item->valueint == 1)) {
        HeatValue = Heat_item->valueint;
        printf("HeatValue set to: %d\n", HeatValue);
    } else {
        printf("Invalid HeatValue (must be 0 or 1)\n");
        parse_success = false;
    }
    
    // ����FanValueֵ (ֻ����0��1)
    cJSON* Fan_item = cJSON_GetObjectItem(root, "FanValue");
    if (cJSON_IsNumber(Fan_item) && (Fan_item->valueint == 0 || Fan_item->valueint == 1)) {
        FanValue = Fan_item->valueint;
        printf("FanValue set to: %d\n", FanValue);
    } else {
        printf("Invalid FanValue (must be 0 or 1)\n");
        parse_success = false;
    }

    cJSON_Delete(root);
	return parse_success;
}


void loraControl(int LightValue, int WaterPumpValue, int MotorValue, int DoorValue, int HeatValue, int FanValue)
{
	set_realy_data(DEVICE_LIGHT, LightValue);
	set_realy_data(DEVICE_PUMP, WaterPumpValue);
	set_realy_data(DEVICE_MOTOR_CUT, MotorValue);
	set_realy_data(DEVICE_HEAT, HeatValue);
	set_realy_data(DEVICE_MOTOR_FAN, FanValue);
	
	//set_boor_flag(DoorValue);
	
	// �ſ����߼�
    DoorState current_state = get_door_state();
    
    if (DoorValue == 0) {  // ��������
        // ֻ���Ų��ǹر�״̬�Ҳ����ƶ�ʱ��ִ�й���
        if (current_state != DOOR_CLOSED && current_state != DOOR_MOVING) {
            close_door();
        }
    } else if (DoorValue == 1) {  // ��������
        // ֻ���Ų��Ǵ�״̬�Ҳ����ƶ�ʱ��ִ�п���
        if (current_state != DOOR_OPEN && current_state != DOOR_MOVING) {
            open_door();
        }
    }
}


