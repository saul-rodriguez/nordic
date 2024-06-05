#include "aska.h"

void ASKA_write_reg(uint8_t add, uint32_t value)
{

    static uint8_t tx_buffer[5];
	static uint8_t rx_buffer[5];

    tx_buffer[0] = add;
    
    tx_buffer[1] = (uint8_t)(value >> 24) & 0xff;
    tx_buffer[2] = (uint8_t)(value >> 16) & 0xff;
    tx_buffer[3] = (uint8_t)(value >> 8) & 0xff;
    tx_buffer[4] = (uint8_t)(value >> 0) & 0xff;

	SPI_transaction(tx_buffer,5);

}