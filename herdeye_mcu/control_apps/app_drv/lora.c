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

volatile bool effect_running = false; // 控制效果循环是否运行
volatile uint8_t current_mode = 0xFF; // 记录当前模式

void lora_init(){
	UART_Init_Type uart_init;

	// USART 初始化设置
	uart_init.ClockFreqHz   = CLOCK_APB2_FREQ;
	uart_init.BaudRate 			= 9600u;										//串口波特率
	uart_init.WordLength    = UART_WordLength_8b;						//字长为8位数据格式
	uart_init.StopBits      = UART_StopBits_1;							//一个停止位
	uart_init.Parity        = UART_Parity_None;								//无奇偶校验位
	uart_init.XferMode      = UART_XferMode_RxTx;				        //收发模式
	uart_init.HwFlowControl = UART_HwFlowControl_None;                  //无硬件数据流控制	
	uart_init.EnableSwapTxRxXferSignal = false;
	uart_init.XferSignal    = UART_XferSignal_Normal;
	
	
	UART_Init(UART2, &uart_init);										//初始化串口2
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
        
        // 防止溢出，保留最后一位给结束符
        if (lora_rx_index < (LORA_RX_BUF_SIZE - 1)) {
            lora_uart_rx_buf[lora_rx_index++] = data;
            
            // 检测帧结束符（假设以\n结尾）
            if (data == '\n') {
                lora_uart_rx_buf[lora_rx_index] = '\0'; // 添加结束符
                lora_frame_ready = true; // 通知主循环
            }
        }
        UART_ClearInterruptStatus(UART2, UART_INT_RX_DONE);
    }
}

void lora_ProcessData() {
    if (lora_frame_ready) {
        // 进入临界区，禁止中断防止数据修改
        bool irq = __get_PRIMASK();
        __disable_irq();
        
        printf("Received: %s\r\n", lora_uart_rx_buf);
        
        // 解析JSON数据
        bool parse_success = Parse_JSON_Data((const char*)lora_uart_rx_buf);
        
        if (parse_success) {
            // 发送ACK响应
            char ack_msg[32];
            snprintf(ack_msg, sizeof(ack_msg), "ACK%d\r\n", SequenceNumber);
            lora_uart_putstr((uint8_t *)ack_msg);
            
            // 执行控制
            loraControl(LightValue, WaterPumpValue, MotorValue, DoorValue, HeatValue, FanValue);
            
            printf("Control applied: L:%d WP:%d M:%d D:%d\n", 
                   LightValue, WaterPumpValue, MotorValue, DoorValue);
        } else {
            printf("JSON parse failed, skipping control\n");
        }
        
        // 处理完成后重置状态
        lora_rx_index = 0;
        lora_frame_ready = false;
        
        // 恢复中断状态
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


    // 解析LightValue值 (只接受0或1)
    cJSON* Light_item = cJSON_GetObjectItem(root, "LightValue");
    if (cJSON_IsNumber(Light_item) && (Light_item->valueint == 0 || Light_item->valueint == 1)) {
        LightValue = Light_item->valueint;
        printf("LightValue set to: %d\n", LightValue);
    } else {
        printf("Invalid LightValue (must be 0 or 1)\n");
		parse_success = false;
    }
    
    // 解析WaterPumpValue值 (只接受0或1)
    cJSON* WaterPump_item = cJSON_GetObjectItem(root, "WaterPumpValue");
    if (cJSON_IsNumber(WaterPump_item) && (WaterPump_item->valueint == 0 || WaterPump_item->valueint == 1)) {
        WaterPumpValue = WaterPump_item->valueint;
        printf("WaterPumpValue set to: %d\n", WaterPumpValue);
    } else {
        printf("Invalid WaterPumpValue (must be 0 or 1)\n");
		parse_success = false;
    }

    // 解析MotorValue值 (只接受0或1)
    cJSON* Motor_item = cJSON_GetObjectItem(root, "MotorValue");
    if (cJSON_IsNumber(Motor_item) && (Motor_item->valueint == 0 || Motor_item->valueint == 1)) {
        MotorValue = Motor_item->valueint;
        printf("MotorValue set to: %d\n", MotorValue);
    } else {
        printf("Invalid MotorValue (must be 0 or 1)\n");
		parse_success = false;
    }

    // 解析DoorValue值 (只接受0或1)
    cJSON* Door_item = cJSON_GetObjectItem(root, "DoorValue");
    if (cJSON_IsNumber(Door_item) && (Door_item->valueint == 0 || Door_item->valueint == 1)) {
        DoorValue = Door_item->valueint;
        printf("DoorValue set to: %d\n", DoorValue);
    } else {
        printf("Invalid DoorValue (must be 0 or 1)\n");
		parse_success = false;
    }
	
	// 解析HeatValue值 (只接受0或1)
    cJSON* Heat_item = cJSON_GetObjectItem(root, "HeatValue");
    if (cJSON_IsNumber(Heat_item) && (Heat_item->valueint == 0 || Heat_item->valueint == 1)) {
        HeatValue = Heat_item->valueint;
        printf("HeatValue set to: %d\n", HeatValue);
    } else {
        printf("Invalid HeatValue (must be 0 or 1)\n");
        parse_success = false;
    }
    
    // 解析FanValue值 (只接受0或1)
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
	
	// 门控制逻辑
    DoorState current_state = get_door_state();
    
    if (DoorValue == 0) {  // 关门命令
        // 只有门不是关闭状态且不在移动时才执行关门
        if (current_state != DOOR_CLOSED && current_state != DOOR_MOVING) {
            close_door();
        }
    } else if (DoorValue == 1) {  // 开门命令
        // 只有门不是打开状态且不在移动时才执行开门
        if (current_state != DOOR_OPEN && current_state != DOOR_MOVING) {
            open_door();
        }
    }
}


