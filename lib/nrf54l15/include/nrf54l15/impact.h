/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_IMPACT_H
#define NRF54L15_SUPPORT_IMPACT_H
#include <stdbool.h>
#include <stdint.h>
#include <nrf54l15/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Magnitude threshold detector with hysteresis and a refractory period.
 * This is a candidate event detector, not a validated archery classifier.
 */
struct nrf54l15_impact {
	float trigger_mps2;
	float release_mps2;
	uint32_t refractory_ms;
	uint32_t last_event_ms;
	bool latched;
	bool has_event;
};

int nrf54l15_impact_init(struct nrf54l15_impact *state, float trigger_mps2,
			float release_mps2, uint32_t refractory_ms);
/* Returns 1 for a new event, 0 otherwise, negative errno for invalid input.
 * First event may occur at boot. A sustained high level only fires once.
 * Events suppressed by the refractory interval require a new low/high cycle.
 * now_ms must be monotonic modulo uint32_t; gaps must be less than 2^32 ms.
 */
int nrf54l15_impact_update(struct nrf54l15_impact *state,
			  const struct nrf54l15_vec3 *accel, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
