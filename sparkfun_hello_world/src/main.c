/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#define SLEEP_TIME   100

#define LED5_NODE DT_ALIAS(led5)
//#define LED5_NODE DEVICE_DT_GET(led5)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED5_NODE, gpios);

void main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	printk("Hello World! %s\n", CONFIG_BOARD);


	while (1) {
		gpio_pin_set_dt(&led,1);
		k_msleep(SLEEP_TIME);
		gpio_pin_set_dt(&led,0);
		k_msleep(SLEEP_TIME);
	}
}

