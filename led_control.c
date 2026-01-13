#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define LED_STEADY_NODE DT_ALIAS(indicator_led_steady)
#define LED_PULSE_NODE  DT_ALIAS(indicator_led_pulse)

#define LED_TIMEOUT_MS CONFIG_ZMK_DUMB_LED_TIMEOUT

/* Breathing parameters */
#define BREATH_MIN_BRIGHTNESS 20
#define BREATH_MAX_BRIGHTNESS 100
#define BREATH_STEP 3
#define PWM_PERIOD_US 5000 /* 200 Hz to keep CPU load low */
#define BREATH_INTERVAL_MIN_MS 20   /* fast when typing */
#define BREATH_INTERVAL_MAX_MS 200  /* slow when idle */
#define TYPING_DECAY_MS 600         /* how long before slowing back down */

/* Activity tracking */
static bool leds_are_active = true;
static struct k_timer activity_timeout_timer;
static struct k_timer typing_decay_timer;

/* Steady LED */
#ifdef CONFIG_ZMK_DUMB_LED_STEADY
#if DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay)
static const struct gpio_dt_spec led_steady = GPIO_DT_SPEC_GET(LED_STEADY_NODE, gpios);
static bool steady_led_initialized = false;
#endif
#endif

/* Pulse LED (GPIO + software PWM) */
#ifdef CONFIG_ZMK_DUMB_LED_PULSE
#if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
static const struct gpio_dt_spec led_pulse = GPIO_DT_SPEC_GET(LED_PULSE_NODE, gpios);
static struct k_timer pulse_timer;
static struct k_timer pwm_on_timer;
static struct k_timer pwm_off_timer;
static uint8_t pulse_brightness = BREATH_MAX_BRIGHTNESS;
static int8_t brightness_direction = -1;
static uint32_t pwm_on_time_us = PWM_PERIOD_US;
static uint32_t breath_interval_ms = BREATH_INTERVAL_MAX_MS;
static bool pulse_led_initialized = false;
static bool typing_active = false;
#endif
#endif

static void activity_timeout_handler(struct k_timer *timer)
{
    leds_are_active = false;

#ifdef CONFIG_ZMK_DUMB_LED_STEADY
#if DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay)
    gpio_pin_set_dt(&led_steady, 0);
#endif
#endif

#ifdef CONFIG_ZMK_DUMB_LED_PULSE
#if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
    gpio_pin_set_dt(&led_pulse, 0);
    pulse_brightness = BREATH_MIN_BRIGHTNESS;
    brightness_direction = 1;
    breath_interval_ms = BREATH_INTERVAL_MAX_MS;
    typing_active = false;
    k_timer_stop(&pulse_timer);
    k_timer_stop(&pwm_on_timer);
    k_timer_stop(&pwm_off_timer);
    k_timer_stop(&typing_decay_timer);
#endif
#endif

    LOG_INF("LEDs turned off after timeout");
}

#ifdef CONFIG_ZMK_DUMB_LED_PULSE
#if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
static void pwm_on_handler(struct k_timer *timer)
{
    if (!leds_are_active) {
        gpio_pin_set_dt(&led_pulse, 0);
        return;
    }

    gpio_pin_set_dt(&led_pulse, 1);
    k_timer_start(&pwm_off_timer, K_USEC(pwm_on_time_us), K_NO_WAIT);
}

static void pwm_off_handler(struct k_timer *timer)
{
    if (!leds_are_active) {
        return;
    }

    gpio_pin_set_dt(&led_pulse, 0);
    k_timer_start(&pwm_on_timer, K_USEC(PWM_PERIOD_US - pwm_on_time_us), K_NO_WAIT);
}

static void typing_decay_handler(struct k_timer *timer)
{
    typing_active = false;
    breath_interval_ms = BREATH_INTERVAL_MAX_MS;
    k_timer_stop(&pulse_timer);
    k_timer_start(&pulse_timer, K_MSEC(breath_interval_ms), K_MSEC(breath_interval_ms));
    LOG_DBG("Breathing slowed after typing stopped");
}

static void pulse_handler(struct k_timer *timer)
{
    if (!leds_are_active) {
        return;
    }

    pulse_brightness += (BREATH_STEP * brightness_direction);

    if (pulse_brightness >= BREATH_MAX_BRIGHTNESS) {
        pulse_brightness = BREATH_MAX_BRIGHTNESS;
        brightness_direction = -1;
    } else if (pulse_brightness <= BREATH_MIN_BRIGHTNESS) {
        pulse_brightness = BREATH_MIN_BRIGHTNESS;
        brightness_direction = 1;
    }

    pwm_on_time_us = (PWM_PERIOD_US * pulse_brightness) / 100;
    LOG_DBG("LED breath: %d%% interval: %ums", pulse_brightness, breath_interval_ms);
}
#endif
#endif

static void reset_activity_timer(void)
{
    if (!leds_are_active) {
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
            breath_interval_ms = BREATH_INTERVAL_MAX_MS;
            typing_active = false;
            gpio_pin_set_dt(&led_pulse, 1);
            k_timer_start(&pwm_on_timer, K_NO_WAIT, K_NO_WAIT);
            k_timer_start(&pulse_timer, K_MSEC(breath_interval_ms), K_MSEC(breath_interval_ms));
        }
#endif
#endif
        leds_are_active = true;
        LOG_INF("LEDs turned back on by key press");
    }

#ifdef CONFIG_ZMK_DUMB_LED_PULSE
#if DT_NODE_HAS_STATUS(LED_PULSE_NODE, okay)
    if (pulse_led_initialized) {
        breath_interval_ms = BREATH_INTERVAL_MIN_MS;
        typing_active = true;
        k_timer_stop(&pulse_timer);
        k_timer_start(&pulse_timer, K_MSEC(breath_interval_ms), K_MSEC(breath_interval_ms));
    }
    k_timer_stop(&typing_decay_timer);
    k_timer_start(&typing_decay_timer, K_MSEC(TYPING_DECAY_MS), K_NO_WAIT);
#endif
#endif

    k_timer_stop(&activity_timeout_timer);
    k_timer_start(&activity_timeout_timer, K_MSEC(LED_TIMEOUT_MS), K_NO_WAIT);
}

static int led_control_event_listener(const zmk_event_t *eh)
{
    const struct zmk_position_state_changed *pos_ev = as_zmk_position_state_changed(eh);

    if (pos_ev != NULL && pos_ev->state) {
        reset_activity_timer();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_control, led_control_event_listener);
ZMK_SUBSCRIPTION(led_control, zmk_position_state_changed);

static int led_control_init(void)
{
#ifdef CONFIG_ZMK_DUMB_LED_STEADY
#if DT_NODE_HAS_STATUS(LED_STEADY_NODE, okay)
    if (!device_is_ready(led_steady.port)) {
        LOG_ERR("Steady indicator LED port not ready");
    } else {
        int ret = gpio_pin_configure_dt(&led_steady, GPIO_OUTPUT_ACTIVE);
        if (ret == 0) {
            ret = gpio_pin_set_dt(&led_steady, 1);
        }
        if (ret != 0) {
            LOG_ERR("Failed to init steady indicator LED");
        } else {
            steady_led_initialized = true;
            LOG_INF("Steady indicator LED initialized");
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
        if (ret == 0) {
            ret = gpio_pin_set_dt(&led_pulse, 1);
        }
        if (ret != 0) {
            LOG_ERR("Failed to init pulse indicator LED");
        } else {
            pulse_led_initialized = true;
            k_timer_init(&pwm_on_timer, pwm_on_handler, NULL);
            k_timer_init(&pwm_off_timer, pwm_off_handler, NULL);
            k_timer_init(&pulse_timer, pulse_handler, NULL);
            k_timer_init(&typing_decay_timer, typing_decay_handler, NULL);
            breath_interval_ms = BREATH_INTERVAL_MAX_MS;
            k_timer_start(&pwm_on_timer, K_NO_WAIT, K_NO_WAIT);
            k_timer_start(&pulse_timer, K_MSEC(breath_interval_ms), K_MSEC(breath_interval_ms));
            LOG_INF("Pulse indicator LED initialized with responsive breathing");
        }
    }
#endif
#endif

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