/*
 * Copyright 2021 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board_init.h"

/*
 * Macros.
 */

/*
 * Variables.
 */

/*
 * Declarations.
 */

/*
 * Functions.
 */
int main(void)
{
    BOARD_Init();

    printf("startsart\r\n");
    

    while (1)
    {
		
		lora_ProcessData();
		send_sensors_data();
		
		//delay_ms(500);
    }
}



/* EOF. */