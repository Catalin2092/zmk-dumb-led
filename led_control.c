#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// Bind to the indicator-led alias defined in your shield DTS.
#define LED_NODE DT_ALIAS(indicator_led)

#if DT_NODE_HAS_STATUS(LED_NODE, okay)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static struct k_timer led_timeout_timer;
static bool led_is_active = true;

// 2 minutes in milliseconds
#define LED_TIMEOUT_MS CONFIG_ZMK_DUMB_LED_TIMEOUT

static void led_timeout_handler(struct k_timer *timer) {
    gpio_pin_set_dt(&led, 0);  // Turn LED off
    led_is_active = false;
    LOG_INF("LED turned off after timeout");
}

static void reset_led_timer(void) {
    if (!led_is_active) {
        // Turn LED back on if it was off
        gpio_pin_set_dt(&led, 1);
        led_is_active = true;
        LOG_INF("LED turned back on by key press");
    }
    
    // Reset the timeout timer
    k_timer_stop(&led_timeout_timer);
    k_timer_start(&led_timeout_timer, K_MSEC(LED_TIMEOUT_MS), K_NO_WAIT);
}

// Event listener for key presses
static int led_control_event_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *pos_ev = as_zmk_position_state_changed(eh);
    
    if (pos_ev != NULL && pos_ev->state) {  // Key pressed (not released)
        reset_led_timer();
    }
    
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_control, led_control_event_listener);
ZMK_SUBSCRIPTION(led_control, zmk_position_state_changed);

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

    // Initialize and start the timeout timer
    k_timer_init(&led_timeout_timer, led_timeout_handler, NULL);
    k_timer_start(&led_timeout_timer, K_MSEC(LED_TIMEOUT_MS), K_NO_WAIT);

    LOG_INF("Indicator LED initialized with auto-off timeout");
    return 0;
}

SYS_INIT(led_control_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#else
#warning "indicator-led alias not found; ZMK_LED_CONTROL is inactive."
#endif