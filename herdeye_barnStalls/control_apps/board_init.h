/*
 * Copyright 2022 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef __BOARD_INIT_H__
#define __BOARD_INIT_H__

#include <stdio.h>
#include <stdint.h>

#include "hal_common.h"
#include "hal_rcc.h"
#include "hal_uart.h"
#include "hal_spi.h"
#include "hal_dma.h"
#include "hal_dma_request.h"
#include "hal_gpio.h"
#include "hal_adc.h"
#include "hal_tim.h"

#include "clock_init.h"
#include "pin_init.h"


#include "relay.h"
#include "water_level.h"
#include "door.h"
#include "lora.h"
#include "weight.h"


/* DEBUG UART. */
#define BOARD_DEBUG_UART_PORT        UART1
#define BOARD_DEBUG_UART_BAUDRATE    9600u
#define BOARD_DEBUG_UART_FREQ        CLOCK_APB2_FREQ

void delay_us(uint32_t us);

void delay_ms(uint32_t nms);

void BOARD_Init(void);



#endif /* __BOARD_INIT_H__ */
