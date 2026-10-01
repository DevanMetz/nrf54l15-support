/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_DMIC_H
#define NRF54L15_SUPPORT_DMIC_H
#include <stddef.h>
#include <stdint.h>
#include <zephyr/audio/dmic.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_dmic_config {
	uint32_t sample_rate_hz; /* Requested rate: 8000..48000, actual clock may differ. */
	uint32_t min_pdm_clock_hz;
	uint32_t max_pdm_clock_hz;
	uint16_t samples_per_block; /* Even, 2..32766 (16-bit mono, word-aligned blocks). */
	enum pdm_lr channel;
};

enum nrf54l15_dmic_state {
	NRF54L15_DMIC_IDLE = 0,
	NRF54L15_DMIC_RUNNING,
	NRF54L15_DMIC_STOPPING,
};

struct nrf54l15_dmic {
	const struct device *dev;
	struct k_mem_slab *slab;
	uint16_t block_bytes;
	enum nrf54l15_dmic_state state;
};

/* Zero-initialize before first use; exclusively own both the peripheral and
 * an initialized RAM slab with >=4 blocks, each exactly samples_per_block*2
 * bytes, aligned to at least 4 bytes. Keep state/slab/backing memory alive
 * until stop succeeds (static storage is recommended). Do not directly call
 * dmic_* or allocate/free this slab while the adapter owns it.
 * NCS v3.3.0 nrfx PDM backend, mono PCM16, standard 40..60% clock duty limits.
 * Serialize every call in thread context. No heap allocation or callbacks.
 * On an error after configuration, state is STOPPING: call stop before retry.
 */
int nrf54l15_dmic_start(struct nrf54l15_dmic *mic, const struct device *dev,
	struct k_mem_slab *slab, const struct nrf54l15_dmic_config *config);

/* Copy one whole block to caller-owned PCM storage, then return the DMA block
 * to the slab. Capacity is in int16_t samples; it may exceed the block size.
 * Output remains unchanged on failure. timeout_ms >=0 (0 is nonblocking).
 * Queue timeout/empty and driver errors propagate. A timeout does not prove
 * hardware is still running: nrfx can stop after a buffer/queue overrun.
 */
int nrf54l15_dmic_read(struct nrf54l15_dmic *mic, int16_t *samples,
	size_t capacity, int32_t timeout_ms);

/* Stop, discard queued blocks, wait for in-flight DMA buffers, then disable
 * the stream to confirm the nrfx backend is inactive. timeout_ms >=0; zero
 * makes one nonblocking attempt. Returns -ETIMEDOUT if cleanup is incomplete.
 * Every error leaves STOPPING; retain all storage and retry stop. Never clear
 * the state, reuse the slab, or restart on an unsuccessful stop. IDLE is a no-op.
 */
int nrf54l15_dmic_stop(struct nrf54l15_dmic *mic, int32_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif
