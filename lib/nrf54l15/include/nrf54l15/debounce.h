/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_DEBOUNCE_H
#define NRF54L15_SUPPORT_DEBOUNCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum nrf54l15_button_event {
	NRF54L15_BUTTON_NONE = 0,
	NRF54L15_BUTTON_PRESSED = 1,
	NRF54L15_BUTTON_RELEASED = 2,
};

/* Initialize to {.debounce_ms = 20}. Other fields must initially be zero.
 * Poll with a monotonic millisecond clock; uint32_t wrap is supported.
 * A button held at startup produces one press after the debounce interval.
 */
struct nrf54l15_debounce {
	uint32_t debounce_ms;
	uint32_t changed_ms;
	bool candidate;
	bool stable;
};

enum nrf54l15_button_event nrf54l15_debounce_update(
	struct nrf54l15_debounce *state, bool pressed, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
