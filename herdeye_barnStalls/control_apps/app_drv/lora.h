#ifndef __LORA_H__
#define __LORA_H__

#include "board_init.h"


extern int LightValue;
extern int WaterPumpValue;
extern int MotorValue;
extern int DoorValue;

void lora_init();
void lora_ProcessData();
bool Parse_JSON_Data(const char* json_str);
void loraControl(int LightValue,int WaterPumpValue,int MotorValue,int DoorValue);

void lora_uart_putchar(uint8_t c);

uint8_t lora_uart_getchar(void);

void lora_uart_putstr(uint8_t *str);

void Control();
#endif
