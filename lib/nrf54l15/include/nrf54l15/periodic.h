/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_PERIODIC_H
#define NRF54L15_SUPPORT_PERIODIC_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Rational-rate deadlines on a monotonic 64-bit tick clock. Each deadline is
 * rounded UP to the next tick, without accumulating rounding drift. The first
 * sample is one period after start_ticks. Caller owns/serializes the state.
 */
struct nrf54l15_periodic {
	uint64_t start_ticks;
	uint64_t next_slot;
	uint32_t tick_hz;
	uint32_t sample_hz;
};

int nrf54l15_periodic_init(struct nrf54l15_periodic *clock, uint32_t tick_hz,
			 uint32_t sample_hz, uint64_t start_ticks);
int nrf54l15_periodic_deadline(const struct nrf54l15_periodic *clock, uint64_t *ticks);
/* Reports how many scheduled slots have become due (0 if called early), then
 * advances to the next future deadline. A value >1 represents missed slots;
 * callers should read one current sample instead of replaying stale samples.
 * Invalid inputs/overflow leave state and output unchanged. Clock wrap is not
 * supported; ordinary Zephyr uptime ticks remain usable for centuries.
 */
int nrf54l15_periodic_consume(struct nrf54l15_periodic *clock, uint64_t now_ticks,
			    uint64_t *elapsed_slots);

#ifdef __cplusplus
}
#endif
#endif
