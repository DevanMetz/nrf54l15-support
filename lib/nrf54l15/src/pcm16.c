/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <nrf54l15/pcm16.h>

int nrf54l15_pcm16_measure(const int16_t *samples, uint32_t count,
	struct nrf54l15_pcm16_levels *levels)
{
	if (!samples || !count || !levels) {
		return -EINVAL;
	}
	int64_t sum = 0;
	uint64_t squares = 0;
	struct nrf54l15_pcm16_levels result = {0};
	for (uint32_t i = 0; i < count; ++i) {
		int32_t value = samples[i];
		uint16_t magnitude = (uint16_t)(value < 0 ? -value : value);
		sum += value;
		squares += (uint64_t)(value * value);
		if (magnitude > result.peak) {
			result.peak = magnitude;
		}
		if (value == INT16_MIN || value == INT16_MAX) {
			++result.rail_samples;
		}
	}
	result.mean = (float)sum / (float)count;
	result.rms = sqrtf((float)squares / (float)count);
	*levels = result;
	return 0;
}
