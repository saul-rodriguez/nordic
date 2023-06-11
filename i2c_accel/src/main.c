/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/i2c.h>
//#include "mcp23017.h"
#include "ism330dhcx_reg.h"

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
//static int16_t data_raw_temperature;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
//static float temperature_degC;
static uint8_t whoamI, rst;
//static uint8_t tx_buffer[1000];

#define SLEEP_MS 50

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

#define I2C0_NODE DT_NODELABEL(mysensor)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

void main(void)
{
	uint8_t portb_val;
	uint8_t option;
	stmdev_ctx_t dev_ctx;
	
	printk("I2C Accel Tests\n");

	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);		
	}


	  /* Initialize mems driver interface */
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;

	k_msleep(SLEEP_MS);
	/* Check device ID */
	ism330dhcx_device_id_get(&dev_ctx, &whoamI);
	printk("ISM whoiam %x\n", whoamI);

/* Restore default configuration */
    ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        ism330dhcx_reset_get(&dev_ctx, &rst);
        printk("ISM resetting dev %x\n", rst);
    } while (rst);

	option = 'a';

	 /* Start device configuration. */
      ism330dhcx_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
      /* Enable Block Data Update */
      ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
      /* Set Output Data Rate */
      ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_104Hz);
      ism330dhcx_gy_data_rate_set(&dev_ctx, ISM330DHCX_GY_ODR_104Hz);
      /* Set full scale */
      ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_2g);
      ism330dhcx_gy_full_scale_set(&dev_ctx, ISM330DHCX_2000dps);
      /* Configure filtering chain(No aux interface)
       *
       * Accelerometer - LPF1 + LPF2 path
       */
      ism330dhcx_xl_hp_path_on_out_set(&dev_ctx, ISM330DHCX_LP_ODR_DIV_100);
      ism330dhcx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);

      //ism330dhcx_g
      ism330dhcx_gy_lp1_bandwidth_set(&dev_ctx, ISM330DHCX_MEDIUM);
      ism330dhcx_gy_filter_lp1_set(&dev_ctx, PROPERTY_ENABLE);

  /* Read samples in polling mode (no int) */
	while(1) {
		 uint8_t reg;

		    /* Read output only if new xl value is available */
		 switch (option) {
		 	 	 	 case	'a':
		 	 	 		 	 	 ism330dhcx_xl_flag_data_ready_get(&dev_ctx, &reg);

		 	 	 		 	 	 if (reg) {
		 	 	 		 	 		 // Read acceleration field data
		 	 	 		 	 		 memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		 	 	 		 	 		 ism330dhcx_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		 	 	 		 	 		 acceleration_mg[0] = ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[0]);
		 	 	 		 	 		 acceleration_mg[1] = ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[1]);
		 	 	 		 	 		 acceleration_mg[2] = ism330dhcx_from_fs2g_to_mg(data_raw_acceleration[2]);
		 	 	 		 	 		 //sprintf((char *)tx_buffer,"Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		 	 	 		 	 		 printf("Accel [mg]:%4.2f\t%4.2f\t%4.2f\r\n", acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
		 	 	 		 	 		 //printf("Accel z [mg]:%4.2f\n", (acceleration_mg[2]+119));

		 	 	 		 	 	 }
		 	 	 		 	 	 break;
		 	 	 	 case 'g':
		 	 	 		 	 	 ism330dhcx_gy_flag_data_ready_get(&dev_ctx, &reg);

		 	 	 				 if (reg) {
		 	 	 				      // Read angular rate field data
		 	 	 				      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		 	 	 				      ism330dhcx_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
		 	 	 				      angular_rate_mdps[0] = ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
		 	 	 				      angular_rate_mdps[1] = ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
		 	 	 				      angular_rate_mdps[2] = ism330dhcx_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
		 	 	 				      printf("Angrate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);

		 	 	 				 }
		 	 	 				 break;


		 	 	 	 default: 	break;
		 }

		 k_msleep(SLEEP_MS);

	}


	while(1) {
		k_msleep(SLEEP_MS);

	}

/*
	Mcp23017_open();
	Mcp23017_setTris(PORTA,0x7f);
	Mcp23017_setTris(PORTB,0xff);
		
	while (1) {
		k_msleep(SLEEP_MS);
		Mcp23017_writePort(PORTA,0x00);
		
		portb_val = Mcp23017_readPort(PORTB);
		printk("PORTB = %x\n",portb_val);

		k_msleep(SLEEP_MS);
		Mcp23017_writePort(PORTA,0xff);
	}*/

}

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

	uint8_t buffer[16];
	uint8_t i,aux;
	int32_t ret; 

	buffer[0] = reg;

	aux = 1;
	for (i = 0; i < len; i++) {
		buffer[aux] = bufp[i];
		aux++;
	}
	//SPI_transaction(buffer, aux);

	ret = i2c_write_dt(&dev_i2c, buffer, aux);

	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
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

	uint8_t buffer[16];
	uint8_t i,aux;
	uint32_t ret;

	//uint8_t reading[1]= {0};
    uint8_t sensor_regs[1];
	
	sensor_regs[0] = reg;
	ret = i2c_write_read_dt(&dev_i2c,&sensor_regs[0],1,bufp,len);

	return 0;
}
