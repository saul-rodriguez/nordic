#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>


//LEDBLUE 
#define LEDBLUE_NODE DT_NODELABEL(ledblue)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LEDBLUE_NODE, gpios);

//BUTTON0
#define BUTTON0_NODE DT_NODELABEL(button0)
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON0_NODE, gpios);

//BUTTON1
#define BUTTON1_NODE DT_NODELABEL(button1)
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(BUTTON1_NODE, gpios);

#define LEDBLUE_SetHigh()   gpio_pin_set_dt(&led,1)
#define LEDBLUE_SetLow()    gpio_pin_set_dt(&led,0)
#define LEDBLUE_Toggle()    gpio_pin_toggle_dt(&led)

#define BUTTON0_getVal()    gpio_pin_get_dt(&button)
#define BUTTON1_getVal()    gpio_pin_get_dt(&button1)

void PIN_MANAGER_Initialize (void);


#endif