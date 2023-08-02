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

#define SLEEP_TIME   500

#define LEDBLUE_NODE DT_ALIAS(ledblue)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LEDBLUE_NODE, gpios);


#define TX_SIZE 16
#define RX_SIZE 16
#define RECEIVE_TIMEOUT 100


//Get the device pointer of the UART hardware 
const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));

// Define the transmission buffer
static uint8_t tx_buf[TX_SIZE] =   {"nRF Connect SDK Fundamentals Course\n\r"
							 "Press 1-3 on your keyboard to toggle LEDS 1-3 on your development kit\n\r"};

//Define the receive buffer
static uint8_t rx_buf[RX_SIZE] = {0};

//Define the callback function for UART
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {

	case UART_RX_RDY:
		if((evt->data.rx.len) == 1){

			if(evt->data.rx.buf[evt->data.rx.offset] == '1') 
				gpio_pin_toggle_dt(&led);
			else if (evt->data.rx.buf[evt->data.rx.offset] == '2')
				gpio_pin_toggle_dt(&led);	
		}
		break;
	case UART_RX_DISABLED:
		uart_rx_enable(dev ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
		break;
		
	default:
		break;
	}
}


void main(void)
{
	//printk("Hello World! %s\n", CONFIG_BOARD);
	int ret;

    // Configure LEDBLUE
	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	//CONFIGURE UART0
	// Verify that the UART device is ready 
	if (!device_is_ready(uart)){
		printk("UART device not ready\r\n");
		return;
	}

// Register the UART callback function 
	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret) {
		return;
	}

// Start receiving by calling uart_rx_enable() and pass it the address of the receive  buffer
	ret = uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
	if (ret) {
		return;
	}

//
	gpio_pin_set_dt(&led,0);
	k_msleep(SLEEP_TIME);
	gpio_pin_set_dt(&led,1);	

	printk("Hello World! %s\n", CONFIG_BOARD);

	uart_tx(uart,tx_buf,sizeof(tx_buf),SYS_FOREVER_MS);

	while (1) {
		
	}

}
