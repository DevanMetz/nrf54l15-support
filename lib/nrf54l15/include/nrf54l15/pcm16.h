/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_PCM16_H
#define NRF54L15_SUPPORT_PCM16_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_pcm16_levels {
	float mean;
	float rms;
	uint16_t peak; /* Absolute amplitude: 0..32768, including INT16_MIN. */
	uint32_t rail_samples; /* Samples exactly INT16_MIN or INT16_MAX. */
};

/* Raw PCM code units, including DC; neither acoustic dB SPL nor DC removal.
 * Integer sums/squares do not overflow even for UINT32_MAX full-scale samples.
 * Caller provides count readable samples; zero count/null arguments return
 * -EINVAL without changing output. Floating results have float precision.
 */
int nrf54l15_pcm16_measure(const int16_t *samples, uint32_t count,
	struct nrf54l15_pcm16_levels *levels);

#ifdef __cplusplus
}
#endif
#endif
