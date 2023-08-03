/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include "pin_manager.h"
#include "uart0.h"
#include "ism330.h"

#define SLEEP_TIME   500

void main(void)
{	
	int ret;

	PIN_MANAGER_Initialize();
	UART0_Initialize();	
	ISM_Initialize();

	uint8_t option;
	option = 'a';

	LEDBLUE_SetHigh();
	k_msleep(SLEEP_TIME);
	LEDBLUE_SetLow();	

	printk("Sparkfun Hello %s\n", CONFIG_BOARD);

	while (1) {
		uint8_t reg;

		    /* Read output only if new xl value is available */
		 switch (rx_character) {
		 	 	 	 case 'a':
					 	printRawAccel();					 
		 	 	 		break;
		 	 	 	 case 'g':
					 	printRawAng();
					 	break;
		 	 	 	 default: 	break;
		 }

		 k_msleep(SLEEP_MS);
		
	}

}
