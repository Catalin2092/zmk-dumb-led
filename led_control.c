#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// Define LED node aliases
#define LED_STEADY_NODE DT_ALIAS(indicator_led_steady)
#define LED_PULSE_NODE DT_ALIAS(indicator_led_pulse)

// LED timeout in milliseconds
#define LED_TIMEOUT_MS CONFIG_ZMK_DUMB_LED_TIMEOUT
#define LED_PULSE_INTERVAL_MS CONFIG_ZMK_DUMB_LED_PULSE_INTERVAL

// Breathing LED parameters
#define BREATH_MIN_BRIGHTNESS 20  // Minimum brightness percentage (0-100)
#define BREATH_MAX_BRIGHTNESS 100 // Maximum brightness percentage (0-100)
#define BREATH_STEP 3             // Brightness step per interval
#define PWM_PERIOD_US 2000        // Software PWM period in microseconds (500Hz)
#define PWM_UPDATE_INTERVAL_MS 50 // How often to update breathing brightness

// Activity tracking
static bool leds_are_active = true;
static struct k_timer activity_timeout_timer;

// Steady LED support
#ifdef CONFIG_ZMK_DUMB_LED_STEADY
#if DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay)
static const struct gpio_dt_spec led_steady = GPIO_DT_SPEC_GET(LED_STEADY_NODE, gpios);
static bool steady_led_initialized = false;
#endif
#endif

// Pulsing LED support
#ifdef CONFIG_ZMK_DUMB_LED_PULSE
#if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
static const struct gpio_dt_spec led_pulse = GPIO_DT_SPEC_GET(LED_PULSE_NODE, gpios);
static struct k_timer pulse_timer;
static struct k_timer pwm_timer;
static uint8_t pulse_brightness = BREATH_MAX_BRIGHTNESS;  // Current brightness (0-100)
static int8_t brightness_direction = -1;  // -1 for decreasing, 1 for increasing
static uint32_t pwm_on_time_us = PWM_PERIOD_US;  // PWM on-time in microseconds
static bool pulse_led_initialized = false;
#endif
#endif

static void activity_timeout_handler(struct k_timer *timer) {
    leds_are_active = false;
    
    #ifdef CONFIG_ZMK_DUMB_LED_STEADY
    #if DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay)
    gpio_pin_set_dt(&led_steady, 0);  // Turn steady LED off
    #endif
    #endif
    
    #ifdef CONFIG_ZMK_DUMB_LED_PULSE
    #if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
    gpio_pin_set_dt(&led_pulse, 0);   // Turn pulse LED off
    pulse_brightness = BREATH_MIN_BRIGHTNESS;
    brightness_direction = 1;
    k_timer_stop(&pulse_timer);
    k_timer_stop(&pwm_timer);
    #endif
    #endif
    
    LOG_INF("LEDs turned off after timeout");
}

#ifdef CONFIG_ZMK_DUMB_LED_PULSE
#if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)

// Software PWM handler - toggles LED at high frequency
static void pwm_handler(struct k_timer *timer) {
    if (!leds_are_active) {
        gpio_pin_set_dt(&led_pulse, 0);
        return;
    }
    
    // Toggle LED off, then schedule it to turn back on after on-time
    gpio_pin_set_dt(&led_pulse, 0);
    k_busy_wait(PWM_PERIOD_US - pwm_on_time_us);
    gpio_pin_set_dt(&led_pulse, 1);
}

// Breathing effect handler - updates brightness gradually
static void pulse_handler(struct k_timer *timer) {
    if (!leds_are_active) {
        return;
    }
    
    // Update brightness based on direction
    pulse_brightness += (BREATH_STEP * brightness_direction);
    
    // Reverse direction at min/max brightness
    if (pulse_brightness >= BREATH_MAX_BRIGHTNESS) {
        pulse_brightness = BREATH_MAX_BRIGHTNESS;
        brightness_direction = -1;
    } else if (pulse_brightness <= BREATH_MIN_BRIGHTNESS) {
        pulse_brightness = BREATH_MIN_BRIGHTNESS;
        brightness_direction = 1;
    }
    
    // Calculate PWM on-time based on brightness
    pwm_on_time_us = (PWM_PERIOD_US * pulse_brightness) / 100;
    LOG_DBG("LED breath: %d%%", pulse_brightness);
}
#endif
#endif

static void reset_activity_timer(void) {
    if (!leds_are_active) {
        // Turn LEDs back on if they were off
        #ifdef CONFIG_ZMK_DUMB_LED_STEADY
        #if DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay)
        if (steady_led_initialized) {
            gpio_pin_set_dt(&led_steady, 1);
        }
        #endif
        #endif
        
        #ifdef CONFIG_ZMK_DUMB_LED_PULSE
        #if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
        if (pulse_led_initialized) {
            pulse_brightness = BREATH_MAX_BRIGHTNESS;
            brightness_direction = -1;
            pwm_on_time_us = PWM_PERIOD_US;
            gpio_pin_set_dt(&led_pulse, 1);
            k_timer_start(&pwm_timer, K_USEC(PWM_PERIOD_US), K_USEC(PWM_PERIOD_US));
            k_timer_start(&pulse_timer, K_MSEC(PWM_UPDATE_INTERVAL_MS), K_MSEC(PWM_UPDATE_INTERVAL_MS));
        }
        #endif
        #endif
        
        leds_are_active = true;
        LOG_INF("LEDs turned back on by key press");
    }
    
    // Reset the activity timeout timer
    k_timer_stop(&activity_timeout_timer);
    k_timer_start(&activity_timeout_timer, K_MSEC(LED_TIMEOUT_MS), K_NO_WAIT);
}

// Event listener for key presses
static int led_control_event_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *pos_ev = as_zmk_position_state_changed(eh);
    
    if (pos_ev != NULL && pos_ev->state) {  // Key pressed (not released)
        reset_activity_timer();
    }
    
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_control, led_control_event_listener);
ZMK_SUBSCRIPTION(led_control, zmk_position_state_changed);

static int led_control_init(void) {
    #ifdef CONFIG_ZMK_DUMB_LED_STEADY
    #if DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay)
    if (!device_is_ready(led_steady.port)) {
        LOG_ERR("Steady indicator LED port not ready");
    } else {
        int ret = gpio_pin_configure_dt(&led_steady, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure steady indicator LED pin");
        } else {
            ret = gpio_pin_set_dt(&led_steady, 1);
            if (ret < 0) {
                LOG_ERR("Failed to turn steady indicator LED on");
            } else {
                steady_led_initialized = true;
                LOG_INF("Steady indicator LED initialized");
            }
        }
    }
    #endif
    #endif
    
    #ifdef CONFIG_ZMK_DUMB_LED_PULSE
    #if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
    if (!device_is_ready(led_pulse.port)) {
        LOG_ERR("Pulse indicator LED port not ready");
    } else {
        int ret = gpio_pin_configure_dt(&led_pulse, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure pulse indicator LED pin");
        } else {
            ret = gpio_pin_set_dt(&led_pulse, 1);
            if (ret < 0) {
                LOG_ERR("Failed to turn pulse indicator LED on");
            } else {
                pulse_led_initialized = true;
                k_timer_init(&pwm_timer, pwm_handler, NULL);
                k_timer_init(&pulse_timer, pulse_handler, NULL);
                k_timer_start(&pwm_timer, K_USEC(PWM_PERIOD_US), K_USEC(PWM_PERIOD_US));
                k_timer_start(&pulse_timer, K_MSEC(PWM_UPDATE_INTERVAL_MS), K_MSEC(PWM_UPDATE_INTERVAL_MS));
                LOG_INF("Pulse indicator LED initialized with breathing effect");
            }
        }
    }
    #endif
    #endif
    
    // Initialize and start the activity timeout timer
    k_timer_init(&activity_timeout_timer, activity_timeout_handler, NULL);
    k_timer_start(&activity_timeout_timer, K_MSEC(LED_TIMEOUT_MS), K_NO_WAIT);
    
    #ifdef CONFIG_ZMK_DUMB_LED_STEADY
    #if DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay)
    LOG_INF("LED control initialized with steady LED");
    #endif
    #endif
    #ifdef CONFIG_ZMK_DUMB_LED_PULSE
    #if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
    LOG_INF("LED control initialized with pulse LED");
    #endif
    #endif
    
    return 0;
}

SYS_INIT(led_control_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#if !DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay) && !DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
#warning "No indicator LED aliases found; ZMK_LED_CONTROL is inactive."
#endif