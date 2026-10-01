/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <nrf54l15/impact.h>

int nrf54l15_impact_init(struct nrf54l15_impact *state, float trigger_mps2,
			float release_mps2, uint32_t refractory_ms)
{
	if (state == NULL || !isfinite(trigger_mps2 * trigger_mps2) ||
	    !isfinite(release_mps2) || release_mps2 < 0 || trigger_mps2 <= release_mps2) {
		return -EINVAL;
	}
	*state = (struct nrf54l15_impact){.trigger_mps2 = trigger_mps2,
		.release_mps2 = release_mps2, .refractory_ms = refractory_ms};
	return 0;
}

int nrf54l15_impact_update(struct nrf54l15_impact *state,
			  const struct nrf54l15_vec3 *accel, uint32_t now_ms)
{
	if (state == NULL || accel == NULL || state->trigger_mps2 <= state->release_mps2 ||
	    !isfinite(accel->x) || !isfinite(accel->y) || !isfinite(accel->z)) {
		return -EINVAL;
	}
	float magnitude2 = accel->x * accel->x + accel->y * accel->y + accel->z * accel->z;
	if (!isfinite(magnitude2)) {
		return -ERANGE;
	}
	if (magnitude2 <= state->release_mps2 * state->release_mps2) {
		state->latched = false;
		return 0;
	}
	if (state->latched || magnitude2 < state->trigger_mps2 * state->trigger_mps2) {
		return 0;
	}
	state->latched = true;
	if (state->has_event && (uint32_t)(now_ms - state->last_event_ms) < state->refractory_ms) {
		return 0;
	}
	state->last_event_ms = now_ms;
	state->has_event = true;
	return 1;
}
