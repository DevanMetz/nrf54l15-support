/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_BUTTON_H
#define NRF54L15_SUPPORT_BUTTON_H

#include <zephyr/drivers/gpio.h>
#include <nrf54l15/debounce.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_button {
	struct gpio_dt_spec gpio;
	struct nrf54l15_debounce debounce;
	bool ready;
};

/* Uses the devicetree GPIO polarity and pull configuration. Single caller. */
int nrf54l15_button_init(struct nrf54l15_button *button,
			const struct gpio_dt_spec *gpio, uint32_t debounce_ms);
/* Returns a nrf54l15_button_event or negative errno on GPIO failure. */
int nrf54l15_button_poll(struct nrf54l15_button *button, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
