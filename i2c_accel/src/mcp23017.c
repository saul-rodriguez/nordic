#include "mcp23017.h"

#include <zephyr/kernel.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define I2C0_NODE DT_NODELABEL(mysensor)

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);

int32_t Mcp23017_open(void)
{
    uint8_t config[2] = {MCP23017_IOCON,0x20};
    int32_t ret;

    if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1;
	}

    ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
    if(ret != 0){
		printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		return -1;
	}

    return 0;

}

int32_t Mcp23017_setTris(uint8_t port, uint8_t tris)
{
   uint8_t config[2];
   int ret;

    if (port == PORTA) {
        config[0] = MCP23017_IODIRA;
        config[1] = tris;

        //Configure I/O portA       
        ret = i2c_write_dt(&dev_i2c, config, sizeof(config));        
        if(ret != 0){
		    printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		    return -1;
        }
	} else if (port == PORTB) {
        config[0] = MCP23017_IODIRB;
        config[1] = tris;

        //Configure I/O portA       
        ret = i2c_write_dt(&dev_i2c, config, sizeof(config));        
        if(ret != 0){
		    printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		    return -1;      
        }        
    }
    
    return 0;
}

int32_t Mcp23017_writePort(uint8_t port, uint8_t data)
{
   uint8_t config[2];
   int ret;

    if (port == PORTA) {
        config[0] = MCP23017_GPIOA;
        config[1] = data;

        //Configure I/O portA       
        ret = i2c_write_dt(&dev_i2c, config, sizeof(config));        
        if(ret != 0){
		    printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		    return -1;
        }
	} else if (port == PORTB) {
        config[0] = MCP23017_GPIOB;
        config[1] = data;

        //Configure I/O portA       
        ret = i2c_write_dt(&dev_i2c, config, sizeof(config));        
        if(ret != 0){
		    printk("Failed to write to I2C device address %x at Reg. %x \n", dev_i2c.addr,config[0]);
		    return -1;      
        }        
    }


    return 0;
}

uint8_t Mcp23017_readPort(uint8_t port)
{
    //uint8_t config[2];
    uint8_t reading[1]= {0};
    uint8_t sensor_regs[1];
   int ret;

    if (port == PORTA) {
        //read I/O portA       

        sensor_regs[0] = MCP23017_GPIOA;
        ret = i2c_write_read_dt(&dev_i2c,&sensor_regs[0],1,&reading[0],1);
        if(ret != 0){
	        printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,sensor_regs[0]);
        }
        
	} else if (port == PORTB) {
        //read I/O portB       

        sensor_regs[0] = MCP23017_GPIOB;
        ret = i2c_write_read_dt(&dev_i2c,&sensor_regs[0],1,&reading[0],1);
        if(ret != 0){
	        printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,sensor_regs[0]);
        }
    }

    return reading[0];    
}

