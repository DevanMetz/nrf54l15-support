/* SPDX-License-Identifier: Apache-2.0 */
#include <stddef.h>
#include <nrf54l15/debounce.h>

enum nrf54l15_button_event nrf54l15_debounce_update(
	struct nrf54l15_debounce *state, bool pressed, uint32_t now_ms)
{
	if (state == NULL) {
		return NRF54L15_BUTTON_NONE;
	}
	if (pressed != state->candidate) {
		state->candidate = pressed;
		state->changed_ms = now_ms;
	}
	if (state->stable == state->candidate ||
	    (uint32_t)(now_ms - state->changed_ms) < state->debounce_ms) {
		return NRF54L15_BUTTON_NONE;
	}
	state->stable = state->candidate;
	return state->stable ? NRF54L15_BUTTON_PRESSED : NRF54L15_BUTTON_RELEASED;
}
