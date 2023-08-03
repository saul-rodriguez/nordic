#include "uart0.h"
#include "pin_manager.h"

//Get the device pointer of the UART hardware 

uint8_t tx_buf[TX_SIZE];
uint8_t rx_buf[RX_SIZE];
uint8_t rx_character = 0;

int UART0_Initialize(void)
{
    int ret;

    // Verify that the UART device is ready 
	if (!device_is_ready(uart)){
		printk("UART device not ready\r\n");
		return 0;
	}

	// Register the UART callback function 
	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret) {
		return 0;
	}

	// Start receiving by calling uart_rx_enable() and pass it the address of the receive  buffer
	ret = uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
	if (ret) {
		return 0;
	}

    return 1;
}

void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {

	case UART_RX_RDY:
		if((evt->data.rx.len) == 1){
			rx_character = evt->data.rx.buf[evt->data.rx.offset];
		}
		break;
	case UART_RX_DISABLED:
		uart_rx_enable(dev ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
		break;
		
	default:
		break;
	}
}
