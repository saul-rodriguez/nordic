#ifndef UART0_H
#define UART0_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#define TX_SIZE 16
#define RX_SIZE 16
#define RECEIVE_TIMEOUT 100

//Get the device pointer of the UART hardware 
static const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));

// Define the transmission buffer
extern uint8_t tx_buf[TX_SIZE];

//Define the receive buffer
extern uint8_t rx_buf[RX_SIZE];

int UART0_Initialize(void);
void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

/*TODO: Add a tx function
 a tx call through the API is done by: 
    uart_tx(uart,tx_buf,sizeof(tx_buf),SYS_FOREVER_MS);
 */

#endif