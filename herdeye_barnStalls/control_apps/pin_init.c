/*
 * Copyright 2022 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "pin_init.h"
#include "hal_rcc.h"
#include "hal_gpio.h"

void BOARD_InitPins(void)
{
    /* PB7 - UART1_TX. */
    GPIO_Init_Type gpio_init;
    gpio_init.Pins  = GPIO_PIN_6;
    gpio_init.PinMode  = GPIO_PinMode_AF_PushPull; //GPIO_PinMode_AF_PushPull
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_init);
    GPIO_PinAFConf(GPIOB, gpio_init.Pins, GPIO_AF_7);

    /* PB6 - UART1_RX. */
    gpio_init.Pins  = GPIO_PIN_7;
    gpio_init.PinMode  = GPIO_PinMode_In_Floating; //GPIO_PinMode_In_Floating
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_init);
    GPIO_PinAFConf(GPIOB, gpio_init.Pins, GPIO_AF_7);

	//******************************LoRa******************************//
	// UART2_TX
	gpio_init.Pins  = GPIO_PIN_2;			   // PA2
	gpio_init.Speed = GPIO_Speed_50MHz;
	gpio_init.PinMode = GPIO_PinMode_AF_PushPull; //复用推挽输出
	GPIO_Init(GPIOA, &gpio_init);
	GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_7);

	// USART2_RX
	gpio_init.Pins = GPIO_PIN_3;			  // PA3
	gpio_init.PinMode = GPIO_PinMode_In_Floating; //浮空输入
	GPIO_Init(GPIOA, &gpio_init);
	GPIO_PinAFConf(GPIOA, gpio_init.Pins, GPIO_AF_7);
	
	//******************************LoRa******************************//
	
	
	//*****************************继电器*****************************//
	gpio_init.Pins  = GPIO_PIN_7;
	gpio_init.PinMode  = GPIO_PinMode_Out_PushPull;
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
	
	gpio_init.Pins  = GPIO_PIN_6;
	gpio_init.PinMode  = GPIO_PinMode_Out_PushPull;
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
	
	gpio_init.Pins  = GPIO_PIN_5;
	gpio_init.PinMode  = GPIO_PinMode_Out_PushPull;
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
	//*****************************继电器*****************************//                                                                                                                                                                                                                                                      


	//*****************************水位******************************//
    gpio_init.Pins  = GPIO_PIN_4;
    gpio_init.PinMode  = GPIO_PinMode_In_PullUp;
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init);
	//*****************************水位******************************//  


	//*****************************压力******************************//
    gpio_init.Pins  = GPIO_PIN_11;//sck
    gpio_init.PinMode  = GPIO_PinMode_Out_PushPull;
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio_init);
	
	gpio_init.Pins  = GPIO_PIN_12;//dt
    gpio_init.PinMode  = GPIO_PinMode_In_PullUp;
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio_init);
	//*****************************压力******************************// 


	
	//******************************门*******************************//
	//* PWM PC8 - TIM3_CH3. 
	gpio_init.Pins  = GPIO_PIN_4;
    gpio_init.PinMode  = GPIO_PinMode_AF_PushPull;
    gpio_init.Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio_init);
    GPIO_PinAFConf(GPIOB, gpio_init.Pins, GPIO_AF_2);
	//******************************门*******************************//

	
}

/* EOF. */
