#include "pin_manager.h"

//static struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LEDBLUE_NODE, gpios);

void PIN_MANAGER_Initialize (void)
{
    int ret;

    /* Configure LEDBLUE */
	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	/* Configure BUTTON0 */
	
	if (!device_is_ready(button.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		return;
	}

	//Configure the interrupt on the button's pin 
	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE );

	/* Configure BUTTON1 */

	if (!device_is_ready(button1.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&button1, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		return;
	}

	//Configure the interrupt on the button's pin 
	ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE );
	
}