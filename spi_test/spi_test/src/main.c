/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
//#include <zephyr/drivers/spi.h>

#include "mcp23s17.h"
#include "spi_master.h"


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   500

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/*
//#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)

// SPI master functionality

const struct device *spi_dev;
//static struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);

struct spi_cs_control spim_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
	.delay = 0,
};




static void spi_init(void)
{
	spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);
	if(!device_is_ready(spi_dev)) {
		printk("SPI master device not ready!\n");
	}
	if(!device_is_ready(spim_cs.gpio.port)){
		printk("SPI master chip select device not ready!\n");
	}
}

static const struct spi_config spi_cfg = {
	//.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
	//			 SPI_MODE_CPOL | SPI_MODE_CPHA,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	//			 SPI_MODE_CPOL | SPI_MODE_CPHA,
	.frequency = 5000000,
	.slave = 0,
	.cs = &spim_cs,
};


static int spi_write_test_msg(void)
{
	static uint8_t counter = 0;
	static uint8_t tx_buffer[2];
	static uint8_t rx_buffer[2];

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(tx_buffer)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(rx_buffer),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};

	// Update the TX buffer with a rolling counter
	tx_buffer[0] = counter++;
	printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[1]);

	// Reset signal
	k_poll_signal_reset(&spi_done_sig);
	
	// Start transaction
	int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
	if(error != 0){
		printk("SPI transceive error: %i\n", error);
		return error;
	}

	// Wait for the done signal to be raised and log the rx buffer
	int spi_signaled, spi_result;
	do{
		k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
	} while(spi_signaled == 0);
	printk("SPI RX: 0x%.2x, 0x%.2x\n", rx_buffer[0], rx_buffer[1]);
	return 0;
}

*/

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

	SPI_init();
	printk("SPI master test started\n");

	MCP23S17_setAddress(0x20);

	MCP23S17_setTrisA(0x7f); // Only A7 activated as output
	MCP23S17_writePortA(0x00);
	MCP23S17_setTrisB(0xff);

/*
	while (1) {
		//MCP23S17_platform_write();
		MCP23S17_writePortA(0x00);
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return;
		}
		k_msleep(SLEEP_TIME_MS);	
		MCP23S17_writePortA(0xff);
		MCP23S17_readPortB();
		k_msleep(SLEEP_TIME_MS);	
			
	}
*/

	while (1) {
		gpio_pin_toggle_dt(&led);
		MCP23S17_transaction();
	}
	
}
