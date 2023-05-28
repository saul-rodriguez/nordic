/* 
 * File:   mcp23s17.h
 * Author: saul
 *
 * Created on August 24, 2020, 9:18 PM
 */

#ifndef MCP23S17_H
#define	MCP23S17_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "spi_master.h"

//Control Reg.
#define MCP23017_IOCON 0x0A
//Pin direction Regs.
#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
//Pull-up resistors Regs.
#define MCP23017_GPPUA 0x0C
#define MCP23017_GPPUB 0x0D
//Port registers
#define MCP23017_GPIOA 0x12
#define MCP23017_GPIOB 0x13
//Interrupt enable registers
#define MCP23017_GPINTENA 0x04
#define MCP23017_GPINTENB 0x05
//Interrupt default compare register
#define MCP23017_DEFVALA 0x06
#define MCP23017_DEFVALB 0x07
//Interrupt control registers
#define MCP23017_INTCONA 0x08
#define MCP23017_INTCONB 0x09
//Interrupt flag registers
#define MCP23017_INTFA 0x0E
#define MCP23017_INTFB 0x0F
//Interrupt capture registers
#define MCP23017_INTCAPA 0x10
#define MCP23017_INTCAPB 0x11
    
extern unsigned char SPI_address;

extern volatile unsigned char SPI_write_val;

typedef enum {
    IDLE,
    SEND1,
    SEND2,
    SEND3
} MCP23S17_COMMAND; 
extern volatile MCP23S17_COMMAND mcp23s17_command;

void MCP23S17_setAddress(uint8_t add);

void MCP23S17_setTrisA(uint8_t tris);
void MCP23S17_writePortA(uint8_t val);

void MCP23S17_setTrisB(uint8_t tris);
void MCP23S17_writePortB(uint8_t val);
uint8_t MCP23S17_readPortB(void);

// Test functions
void MCP23S17_transaction();
void MCP23S17_platform_write();

//void writePortA_nowait(void);


#ifdef	__cplusplus
}
#endif

#endif	/* MCP23S17_H */
