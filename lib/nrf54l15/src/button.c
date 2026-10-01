/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <nrf54l15/button.h>

int nrf54l15_button_init(struct nrf54l15_button *button,
			const struct gpio_dt_spec *gpio, uint32_t debounce_ms)
{
	if (button == NULL) {
		return -EINVAL;
	}
	button->ready = false;
	button->debounce = (struct nrf54l15_debounce){.debounce_ms = debounce_ms};
	if (gpio == NULL || gpio->port == NULL || !gpio_is_ready_dt(gpio)) {
		return -ENODEV;
	}
	button->gpio = *gpio;
	int err = gpio_pin_configure_dt(gpio, GPIO_INPUT);
	if (err == 0) {
		button->ready = true;
	}
	return err;
}

int nrf54l15_button_poll(struct nrf54l15_button *button, uint32_t now_ms)
{
	if (button == NULL) {
		return -EINVAL;
	}
	if (!button->ready) {
		return -ENODEV;
	}
	int value = gpio_pin_get_dt(&button->gpio);
	if (value < 0) {
		/* Do not count time with unknown input as a stable press. */
		button->debounce.candidate = button->debounce.stable;
		button->debounce.changed_ms = now_ms;
		return value;
	}
	return nrf54l15_debounce_update(&button->debounce, value != 0, now_ms);
}
