#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// Bind to the indicator-led alias defined in your shield DTS.
#define LED_NODE DT_ALIAS(indicator_led)

#if DT_NODE_HAS_STATUS(LED_NODE, okay)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

static int led_control_init(void) {
    if (!device_is_ready(led.port)) {
        LOG_ERR("Indicator LED port not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure indicator LED pin");
        return ret;
    }

    ret = gpio_pin_set_dt(&led, 1);
    if (ret < 0) {
        LOG_ERR("Failed to turn indicator LED on");
        return ret;
    }

    LOG_INF("Indicator LED forced on at boot (port %s pin %d)", led.port->name, led.pin);
    return 0;
}

SYS_INIT(led_control_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#else
#warning "indicator-led alias not found; ZMK_LED_CONTROL is inactive."
#endif