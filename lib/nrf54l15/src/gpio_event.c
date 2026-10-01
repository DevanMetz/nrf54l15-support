/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <limits.h>
#include <stddef.h>
#include <zephyr/irq.h>
#include <nrf54l15/gpio_event.h>

#if defined(CONFIG_SMP)
#error "gpio_event uses single-core IRQ exclusion; SMP is not supported"
#endif

static void on_gpio(const struct device *port, struct gpio_callback *callback,
	gpio_port_pins_t pins)
{
	struct nrf54l15_gpio_event *event = (struct nrf54l15_gpio_event *)
		((char *)callback - offsetof(struct nrf54l15_gpio_event, callback));
	if (!event->stats.running || port != event->pin.port || !(pins & callback->pin_mask)) {
		return;
	}
	if (event->stats.activations != UINT32_MAX) { ++event->stats.activations; }
	k_sem_give(&event->pending);
}

int nrf54l15_gpio_event_start(struct nrf54l15_gpio_event *event,
	const struct gpio_dt_spec *pin)
{
	if (!event || !pin || !pin->port || pin->pin >= 32 ||
	    (pin->dt_flags & ~(GPIO_ACTIVE_LOW | GPIO_PULL_UP | GPIO_PULL_DOWN)) ||
	    ((pin->dt_flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) == (GPIO_PULL_UP | GPIO_PULL_DOWN))) {
		return -EINVAL;
	}
	if (event->bound) { return -EALREADY; }
	if (!gpio_is_ready_dt(pin)) { return -ENODEV; }
	const struct gpio_driver_config *config = pin->port->config;
	if (!config || !(config->port_pin_mask & (UINT32_C(1) << pin->pin))) {
		return -EINVAL; /* Zephyr otherwise asserts for unsupported/reserved pins. */
	}
	struct gpio_dt_spec copied_pin = *pin;
	*event = (struct nrf54l15_gpio_event){.pin = copied_pin};
	int err = k_sem_init(&event->pending, 0, 1);
	if (!err) { err = gpio_pin_configure_dt(&event->pin, GPIO_INPUT); }
	if (!err) { err = gpio_pin_interrupt_configure_dt(&event->pin, GPIO_INT_DISABLE); }
	if (err) { return err; }
	gpio_init_callback(&event->callback, on_gpio, UINT32_C(1) << event->pin.pin);
	unsigned int key = irq_lock();
	err = gpio_add_callback(event->pin.port, &event->callback);
	if (!err) {
		event->bound = true;
		event->stats.running = true;
	}
	irq_unlock(key);
	if (!err) {
		err = gpio_pin_interrupt_configure_dt(&event->pin, GPIO_INT_EDGE_TO_ACTIVE);
		if (err) {
			key = irq_lock();
			event->stats.running = false;
			irq_unlock(key);
		}
	}
	return err;
}

int nrf54l15_gpio_event_wait(struct nrf54l15_gpio_event *event, uint32_t timeout_ms)
{
	if (!event || timeout_ms > INT32_MAX) { return -EINVAL; }
	if (!event->bound) { return -ENODEV; }
	if (!event->stats.running) { return -EPIPE; }
	int err = k_sem_take(&event->pending, K_NO_WAIT);
	if (!err) { return 0; }
	if (err != -EBUSY) { return err; }
	int level = gpio_pin_get_dt(&event->pin);
	if (level < 0) { return level; }
	if (level) { return 0; }
	return k_sem_take(&event->pending, K_MSEC(timeout_ms));
}

int nrf54l15_gpio_event_stop(struct nrf54l15_gpio_event *event)
{
	if (!event) { return -EINVAL; }
	if (!event->bound) { return 0; }
	unsigned int key = irq_lock();
	event->stats.running = false;
	irq_unlock(key);
	int err = gpio_pin_interrupt_configure_dt(&event->pin, GPIO_INT_DISABLE);
	if (!err) { err = gpio_pin_configure_dt(&event->pin, GPIO_INPUT); }
	if (err) { return err; }
	key = irq_lock();
	err = gpio_remove_callback(event->pin.port, &event->callback);
	if (!err) { event->bound = false; }
	irq_unlock(key);
	if (!err) { k_sem_reset(&event->pending); }
	return err;
}

int nrf54l15_gpio_event_get_stats(const struct nrf54l15_gpio_event *event,
	struct nrf54l15_gpio_event_stats *stats)
{
	if (!event || !stats) { return -EINVAL; }
	unsigned int key = irq_lock();
	*stats = event->stats;
	irq_unlock(key);
	return 0;
}
