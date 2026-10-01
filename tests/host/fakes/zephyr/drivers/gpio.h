#ifndef TEST_ZEPHYR_GPIO_H
#define TEST_ZEPHYR_GPIO_H
#include <stdint.h>
#include <zephyr/device.h>
#define GPIO_INPUT 1
#define GPIO_ACTIVE_LOW 1U
#define GPIO_ACTIVE_HIGH 0U
#define GPIO_PULL_UP 0x10U
#define GPIO_PULL_DOWN 0x20U
#define GPIO_INT_DISABLE 0U
#define GPIO_INT_EDGE_TO_ACTIVE 0x100U
typedef uint32_t gpio_port_pins_t;
struct gpio_driver_config { gpio_port_pins_t port_pin_mask; };
struct gpio_callback;
typedef void (*gpio_callback_handler_t)(const struct device *, struct gpio_callback *, gpio_port_pins_t);
struct gpio_callback { gpio_callback_handler_t handler; gpio_port_pins_t pin_mask; };
static inline void gpio_init_callback(struct gpio_callback *cb, gpio_callback_handler_t handler,
	gpio_port_pins_t pins) { cb->handler = handler; cb->pin_mask = pins; }
struct gpio_dt_spec { const struct device *port; uint32_t pin; uint32_t dt_flags; };
static inline bool gpio_is_ready_dt(const struct gpio_dt_spec *spec)
{
	return device_is_ready(spec->port);
}
int gpio_pin_configure_dt(const struct gpio_dt_spec *, uint32_t);
int gpio_pin_get_dt(const struct gpio_dt_spec *);
int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *, uint32_t);
int gpio_add_callback(const struct device *, struct gpio_callback *);
int gpio_remove_callback(const struct device *, struct gpio_callback *);
#endif
