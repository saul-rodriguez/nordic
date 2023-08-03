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
}