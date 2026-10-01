/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stddef.h>
#include <nrf54l15/periodic.h>

static int valid(const struct nrf54l15_periodic *clock)
{
	return clock != NULL && clock->sample_hz != 0 &&
	       clock->sample_hz <= clock->tick_hz && clock->next_slot != 0;
}

int nrf54l15_periodic_init(struct nrf54l15_periodic *clock, uint32_t tick_hz,
			 uint32_t sample_hz, uint64_t start_ticks)
{
	if (clock == NULL || sample_hz == 0 || sample_hz > tick_hz) {
		return -EINVAL;
	}
	*clock = (struct nrf54l15_periodic){.start_ticks = start_ticks, .next_slot = 1,
		.tick_hz = tick_hz, .sample_hz = sample_hz};
	return 0;
}

int nrf54l15_periodic_deadline(const struct nrf54l15_periodic *clock, uint64_t *ticks)
{
	if (!valid(clock) || ticks == NULL) {
		return -EINVAL;
	}
	/* Divide before multiplying to avoid overflow after long uptimes. */
	uint64_t seconds = clock->next_slot / clock->sample_hz;
	uint64_t fraction = (clock->next_slot % clock->sample_hz) * clock->tick_hz;
	fraction = (fraction + clock->sample_hz - 1) / clock->sample_hz;
	if (seconds > (UINT64_MAX - clock->start_ticks) / clock->tick_hz) {
		return -ERANGE;
	}
	uint64_t deadline = clock->start_ticks + seconds * clock->tick_hz;
	if (fraction > UINT64_MAX - deadline) {
		return -ERANGE;
	}
	*ticks = deadline + fraction;
	return 0;
}

int nrf54l15_periodic_consume(struct nrf54l15_periodic *clock, uint64_t now_ticks,
			    uint64_t *elapsed_slots)
{
	if (!valid(clock) || elapsed_slots == NULL || now_ticks < clock->start_ticks) {
		return -EINVAL;
	}
	uint64_t elapsed = now_ticks - clock->start_ticks;
	uint64_t due = (elapsed / clock->tick_hz) * clock->sample_hz +
		((elapsed % clock->tick_hz) * clock->sample_hz) / clock->tick_hz;
	if (due < clock->next_slot) {
		*elapsed_slots = 0;
		return 0;
	}
	if (due == UINT64_MAX) {
		return -ERANGE;
	}
	*elapsed_slots = due - clock->next_slot + 1;
	clock->next_slot = due + 1;
	return 0;
}
