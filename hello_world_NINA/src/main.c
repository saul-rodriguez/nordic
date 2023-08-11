/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "pin_manager.h"
#include "ism330.h"

#define SLEEP_MS 200


// Define the callback function
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LEDBLUE_Toggle();
}

// Define a variable of type static struct gpio_callback
static struct gpio_callback button_cb_data;

void main(void)
{
	uint8_t val;

	printk("Hello World from my new board! %s\n", CONFIG_BOARD);

	PIN_MANAGER_Initialize();
	ISM_Initialize();


	// Initialize the gpio_callback variable  
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin)); 	
	
	// Add the callback function by calling gpio_add_callback()
 	gpio_add_callback(button.port, &button_cb_data);

	LEDBLUE_SetHigh();
	k_msleep(SLEEP_MS);	
	LEDBLUE_SetLow();


/*
	//Test ledblue
	while(1) {
		LEDBLUE_SetHigh();
		k_msleep(SLEEP_MS);	
		LEDBLUE_SetLow();
		k_msleep(SLEEP_MS);	

	}
*/

/*
	//Test button as dig. input
	while(1) {
		val = gpio_pin_get_dt(&button);
		if (val) {
			LEDBLUE_SetHigh();
		} else {
			LEDBLUE_SetLow();
		}
		k_msleep(SLEEP_MS);

		printRawAccel();
	}
*/

	//Tetst button as interrupt
	while (1) {
		printRawAccel();
		k_msleep(SLEEP_MS);
	}

}
