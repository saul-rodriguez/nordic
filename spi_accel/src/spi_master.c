#include "spi_master.h"

const struct device *spi_dev;
static struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);

struct spi_cs_control spim_cs = {
	.gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
	.delay = 0,
};

static const struct spi_config spi_cfg = {
	//.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
	//			 SPI_MODE_CPOL | SPI_MODE_CPHA,
	.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
//				 SPI_MODE_CPOL | SPI_MODE_CPHA,
	.frequency = 2000000,
	.slave = 0,
	.cs = &spim_cs,
};

void SPI_init(void)
{
	spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);
	if(!device_is_ready(spi_dev)) {
		printk("SPI master device not ready!\n");
	}
	if(!device_is_ready(spim_cs.gpio.port)){
		printk("SPI master chip select device not ready!\n");
	}
}


/*
 * SPI_write_reg()
 * Write a byte to a slave device.
 * add = 8 bit address
 * reg = 8 bit register
 * value = 8 bit value
 */
void SPI_write_reg(uint8_t add, uint8_t reg, uint8_t value)
{    
    static uint8_t tx_buffer[3];
	static uint8_t rx_buffer[3];

    tx_buffer[0] = add;
    tx_buffer[1] = reg;
    tx_buffer[2] = value;

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

    // Reset signal
	k_poll_signal_reset(&spi_done_sig);

    //Start SPI transaction
    int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
	if(error != 0){
		printk("SPI transceive error: %i\n", error);
		return error;
	}

    // Wait for the done signal to be raised
    int spi_signaled, spi_result;
	do{
		k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
	} while(spi_signaled == 0);
     
}

/*
 * SPI_read_reg()
 * Reads a byte from a SPI slave
 * add = address + read bit (user must set read bit before)
 * reg = register
 * value = pointer to 8-bit variable to store value.
 */
void SPI_read_reg(uint8_t add, uint8_t reg, uint8_t *value)
{

	static uint8_t tx_buffer[3];
	static uint8_t rx_buffer[3];

    tx_buffer[0] = add;
    tx_buffer[1] = reg;
    tx_buffer[2] = 0x00;

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

    // Reset signal
	k_poll_signal_reset(&spi_done_sig);

    //Start SPI transaction
    int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
	if(error != 0){
		printk("SPI transceive error: %i\n", error);
		return error;
	}

    // Wait for the done signal to be raised
    int spi_signaled, spi_result;
	do{
		k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
	} while(spi_signaled == 0);

	*value = rx_buffer[2];
	//printk("SPI RX: %x %x %x\n", rx_buffer[0], rx_buffer[1], rx_buffer[2]);

}

void SPI_transaction(uint8_t* pData, uint16_t Size)
{   
	static uint8_t tx_buffer[20];
	static uint8_t rx_buffer[20];
	uint16_t i;

	for (i = 0; i < Size; i++) {
		tx_buffer[i] = pData[i];
	}

	const struct spi_buf tx_buf = {
		.buf = tx_buffer,
		.len = sizeof(Size)
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};

	struct spi_buf rx_buf = {
		.buf = rx_buffer,
		.len = sizeof(Size),
	};
	const struct spi_buf_set rx = {
		.buffers = &rx_buf,
		.count = 1
	};
    // Reset signal
	k_poll_signal_reset(&spi_done_sig);

    //Start SPI transaction
    int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
	if(error != 0){
		printk("SPI transceive error: %i\n", error);
		return error;
	}

    // Wait for the done signal to be raised
    int spi_signaled, spi_result;
	do{
		k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
	} while(spi_signaled == 0);

	//*value = buffer[2];
	printk("SPI RX: %x %x %x\n", rx_buffer[0], rx_buffer[1], rx_buffer[2]);

}

/*
void SPI_receive(uint8_t* pData, uint16_t Size)
{

}

void SPI_transmit(uint8_t* pData, uint16_t Size)
{

}
*/


//write function for ST devices
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	//TODO: create a buffer data, with data[0] = reg, and data[1] = bufp[0], ...
	// Send len+1 bytes by using SPI_transaction
	uint8_t buffer[16];
	uint8_t i,aux;

	buffer[0] = reg;

	aux = 1;
	for (i = 0; i < len; i++) {
		buffer[aux] = bufp[i];
		aux++;
	}

	SPI_transaction(buffer, aux);

	return 0;
}


//read function for ST devices
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	//TODO: create a buffer data, with data[0] = reg | 0x80, and data[1] = bufp[0], ...
		// Send len+1 bytes by using SPI_transaction

	uint8_t buffer[16];
	uint8_t i,aux;

	buffer[0] = reg | 0x80; //set write bit for ST devices!

	aux = 1;
	for (i = 0; i < len; i++) {
		buffer[aux] = 0x00;
		aux++;
	}

	SPI_transaction(buffer, aux);

	aux = 1;
	for (i = 0; i < len; i++) {
			//buffer[aux] = 0x00;
			bufp[i] = buffer[aux];
			aux++;
	}

	return 0;
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Write(handle, ISM330DHCX_I2C_ADD_H, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_write(handle,  ISM330DHCX_I2C_ADD_H & 0xFE, reg,
               (uint8_t*) bufp, len);
#endif
  return 0;
}
*/

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
#if defined(NUCLEO_F411RE)
  HAL_I2C_Mem_Read(handle, ISM330DHCX_I2C_ADD_H, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
#elif defined(STEVAL_MKI109V3)
  reg |= 0x80;
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, 1000);
  HAL_SPI_Receive(handle, bufp, len, 1000);
  HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
#elif defined(SPC584B_DIS)
  i2c_lld_read(handle, ISM330DHCX_I2C_ADD_H & 0xFE, reg, bufp, len);
#endif
  return 0;
}

*/
