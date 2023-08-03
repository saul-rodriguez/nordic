#ifndef ISM330_H
#define ISM330_H

#include <zephyr/drivers/i2c.h>
#include "ism330dhcx_reg.h"

/* Private variables ---------------------------------------------------------*/
extern int16_t data_raw_acceleration[3];
extern int16_t data_raw_angular_rate[3];
//static int16_t data_raw_temperature;
extern float acceleration_mg[3];
extern float angular_rate_mdps[3];
//static float temperature_degC;
extern uint8_t whoamI, rst;
//static uint8_t tx_buffer[1000];

#define SLEEP_MS 50

extern stmdev_ctx_t dev_ctx;

int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

int ISM_Initialize(void);
void printRawAccel(void);
void printRawAng(void);

#define I2C0_NODE DT_NODELABEL(mysensor)
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

#endif