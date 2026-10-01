/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_GPIO_EVENT_H
#define NRF54L15_SUPPORT_GPIO_EVENT_H
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_gpio_event_stats {
	uint32_t activations; /* Observed callbacks, saturating at UINT32_MAX. */
	bool running;
};

struct nrf54l15_gpio_event {
	struct gpio_dt_spec pin;
	struct gpio_callback callback;
	struct k_sem pending;
	struct nrf54l15_gpio_event_stats stats;
	bool bound;
};

/* Active-level wake notification for a latched sensor IRQ or GPIO input.
 * Single-core nRF54L15, CONFIG_GPIO and normal kernel-aware GPIO interrupts.
 * Zero-initialize state; own the pin/interrupt exclusively. Polarity and
 * optional pull come from pin; other flags and simultaneous pulls are rejected.
 * State must remain alive and unmoved until stop succeeds, including after a
 * failed start: callback installation may have succeeded before arming failed.
 * Serialize every public call in thread context, including waits and stop.
 * The ISR only counts activations and gives a binary semaphore; no user code.
 */
int nrf54l15_gpio_event_start(struct nrf54l15_gpio_event *event,
	const struct gpio_dt_spec *pin);

/* Return 0 if a notification was pending, the pin is currently active, or an
 * activation arrives during the wait. timeout_ms is 0..INT32_MAX; 0 polls.
 * Zephyr returns -EBUSY for an empty poll and -EAGAIN for an expired wait.
 * GPIO read errors propagate; -ENODEV means unbound, -EPIPE means failed start
 * or stop requires cleanup. Notifications coalesce and can be stale. Always
 * check/drain the peripheral in thread context; a wake is not a sample count.
 * An already-active pin returns immediately until its source is cleared.
 */
int nrf54l15_gpio_event_wait(struct nrf54l15_gpio_event *event, uint32_t timeout_ms);

/* Disable IRQ, request input reconfiguration to release the nrfx GPIOTE
 * channel, and detach callback. Keep storage alive and retry on any error.
 * On success storage may be reused and start may run again. No-op unbound.
 * Leaves the pin as an input with its configured pull. Stats survive stop.
 */
int nrf54l15_gpio_event_stop(struct nrf54l15_gpio_event *event);
int nrf54l15_gpio_event_get_stats(const struct nrf54l15_gpio_event *event,
	struct nrf54l15_gpio_event_stats *stats);

#ifdef __cplusplus
}
#endif
#endif
