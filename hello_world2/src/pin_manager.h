#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

//LEDBLUE 
#define LEDBLUE_NODE DT_ALIAS(ledblue)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LEDBLUE_NODE, gpios);

#define LEDBLUE_SetHigh()   gpio_pin_set_dt(&led,1)
#define LEDBLUE_SetLow()    gpio_pin_set_dt(&led,0)
#define LEDBLUE_Toggle()    gpio_pin_toggle_dt(&led);

void PIN_MANAGER_Initialize (void);

#endif