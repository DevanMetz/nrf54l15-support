/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/pcm16.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

int main(void)
{
	int16_t samples[1000] = {0};
	struct nrf54l15_pcm16_levels levels = {.mean = 7, .rms = 8, .peak = 9, .rail_samples = 10};
	struct nrf54l15_pcm16_levels before = levels;
	CHECK(nrf54l15_pcm16_measure(NULL, 1, &levels) == -EINVAL);
	CHECK(nrf54l15_pcm16_measure(samples, 0, &levels) == -EINVAL);
	CHECK(nrf54l15_pcm16_measure(samples, 1, NULL) == -EINVAL);
	CHECK(memcmp(&before, &levels, sizeof(levels)) == 0);
	CHECK(nrf54l15_pcm16_measure(samples, 1000, &levels) == 0);
	CHECK(levels.mean == 0 && levels.rms == 0 && levels.peak == 0 && levels.rail_samples == 0);
	const int16_t pattern[] = {-4, 3, 0, 0};
	CHECK(nrf54l15_pcm16_measure(pattern, 4, &levels) == 0);
	CHECK(levels.mean == -0.25f && levels.rms == 2.5f && levels.peak == 4 && levels.rail_samples == 0);
	for (unsigned int i = 0; i < 1000; ++i) { samples[i] = INT16_MIN; }
	CHECK(nrf54l15_pcm16_measure(samples, 1000, &levels) == 0);
	CHECK(levels.mean == -32768 && levels.rms == 32768 && levels.peak == 32768 && levels.rail_samples == 1000);
	for (unsigned int i = 0; i < 1000; ++i) { samples[i] = INT16_MAX; }
	CHECK(nrf54l15_pcm16_measure(samples, 1000, &levels) == 0);
	CHECK(levels.mean == 32767 && fabsf(levels.rms - 32767) < 0.01f && levels.peak == 32767 && levels.rail_samples == 1000);
	for (unsigned int i = 0; i < 1000; ++i) { samples[i] = i % 2 ? -1000 : 1000; }
	CHECK(nrf54l15_pcm16_measure(samples, 1000, &levels) == 0);
	CHECK(levels.mean == 0 && levels.rms == 1000 && levels.peak == 1000 && levels.rail_samples == 0);
	/* Independently calculate a sine's expected RMS, allowing PCM quantization. */
	for (unsigned int i = 0; i < 1000; ++i) {
		samples[i] = (int16_t)lround(12000.0 * sin(2.0 * 3.141592653589793 * i / 1000.0));
	}
	CHECK(nrf54l15_pcm16_measure(samples, 1000, &levels) == 0);
	CHECK(fabsf(levels.mean) < 0.01f && fabsf(levels.rms - (float)(12000.0 / sqrt(2.0))) < 0.1f);
	CHECK(levels.peak == 12000 && levels.rail_samples == 0);
	const int16_t rails[] = {INT16_MIN, INT16_MAX, INT16_MIN + 1, INT16_MAX - 1};
	CHECK(nrf54l15_pcm16_measure(rails, 4, &levels) == 0);
	CHECK(levels.mean == -0.5f && levels.peak == 32768 && levels.rail_samples == 2);
	puts("pcm16 tests passed");
	return 0;
}
