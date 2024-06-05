#ifndef SPI_ASKA_H
#define	SPI_ASKA_H

#include <stdio.h>
#include <stdint.h>
#include "spi_master.h"

#define ASKA_CONF0 0x00
#define ASKA_CONF1 0x01
#define ASKA_ELE1  0x02
#define ASKA_ELE2  0x03

void ASKA_write_reg(uint8_t add, uint32_t value);



#endif