#ifndef SPI_MASTER_H
#define	SPI_MASTER_H

#include <stdio.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)



void SPI_init(void);
void SPI_write_reg(uint8_t add, uint8_t reg, uint8_t value);
void SPI_read_reg(uint8_t add, uint8_t reg, uint8_t *value);
//void SPI_transmit(uint8_t* pData, uint16_t Size);
//void SPI_receive(uint8_t* pData, uint16_t Size);
void SPI_transaction(uint8_t* pData, uint16_t Size);
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);


#endif /* SPI_MASTER_H*/