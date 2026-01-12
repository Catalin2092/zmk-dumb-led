# Dumb LED

A simple ZMK (Zephyr Keyboard Module) library that provides automatic LED control for keyboard indicator LEDs. The module turns the LED on and off based on keyboard activity, with a configurable timeout period.

## Features

- **Activity-Based Control**: Automatically manages indicator LED state based on keyboard usage
- **Auto-Off Timer**: Turns off the LED after a configurable period of inactivity
- **Auto-On on Activity**: Turns the LED back on when any key is pressed
- **Configurable Timeout**: Customize the inactivity timeout via Kconfig
- **Zephyr Integration**: Built on Zephyr's GPIO driver and logging framework

## Installation

1. Add this module to your ZMK workspace by including it in your `west.yml`:

```yaml
remotes:
  - name: zmkfirmware
    url-base: https://github.com/zmkfirmware
  - name: zmk-dumb-led
    url-base: https://github.com/Catalin2092
projects:
  - name: zmk
    remote: zmkfirmware
    revision: v0.3.0
    import: app/west.yml
  - name: zmk-dumb-led
    remote: Catalin2092
    revision: main
```

## Configuration

### Enable the Module

Add to your keyboard's `<shield>.conf` file:

```kconfig
CONFIG_ZMK_LED_CONTROL=y
```

### Set Timeout (Optional)

To customize the inactivity timeout (default: 120000ms / 2 minutes):

```kconfig
CONFIG_ZMK_LED_CONTROL=y
CONFIG_ZMK_DUMB_LED_TIMEOUT=60000  # 1 minute
```

### Device Tree Setup

Ensure your shield's device tree defines an `indicator-led` alias pointing to your GPIO LED:

```devicetree
aliases {
    indicator-led = &led0;
};

led0: led_0 {
    compatible = "gpio-leds";
    label = "Indicator LED";
    gpios = <&gpio0 7 GPIO_ACTIVE_HIGH>;
};
```
