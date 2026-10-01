/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_WATCHDOG_H
#define NRF54L15_SUPPORT_WATCHDOG_H
#include <stdint.h>
#include <zephyr/drivers/watchdog.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF54L15_WATCHDOG_MAX_CHANNELS 8
#define NRF54L15_WATCHDOG_MAX_TIMEOUT_MS UINT32_C(0x07cfffff)

enum nrf54l15_watchdog_state {
	NRF54L15_WATCHDOG_IDLE = 0,
	NRF54L15_WATCHDOG_RUNNING,
	NRF54L15_WATCHDOG_FAULTED,
};

struct nrf54l15_watchdog {
	const struct device *dev;
	int channel_ids[NRF54L15_WATCHDOG_MAX_CHANNELS];
	uint8_t channel_count;
	enum nrf54l15_watchdog_state state;
};

/* Zero-initialize the state before first use. Own the peripheral exclusively
 * from boot; do not combine this adapter with direct wdt_* calls or task_wdt.
 * Start 1..8 channels with a common timeout, RESET_SOC, and no callback.
 * Options are WDT_OPT_PAUSE_IN_SLEEP and/or WDT_OPT_PAUSE_HALTED_BY_DBG, or 0.
 * Validate before allocating; once SDK setup begins, any error latches FAULTED.
 * Reset the device after a fault: do not zero the state and retry, because
 * some hardware channels may already be allocated or the timer may be armed.
 * RUNNING means setup succeeded; it is not hardware status readback.
 */
int nrf54l15_watchdog_start(struct nrf54l15_watchdog *watchdog,
	const struct device *dev, uint32_t timeout_ms, uint8_t channels, uint8_t options);

/* Feed only the calling worker's logical slot (0..channels-1) after useful
 * progress. All enabled hardware channels must be fed to reload the counter.
 * There are no independent per-worker deadlines and no automatic feeds.
 * Once started, different workers may feed their own slots concurrently.
 * Serialize start/stop with every feed; use thread context for lifecycle calls.
 * A feed error is returned unchanged; the caller decides whether to retry.
 */
int nrf54l15_watchdog_feed(const struct nrf54l15_watchdog *watchdog, uint8_t slot);

/* Explicitly stop a running watchdog. Only a successful SDK disable clears
 * the state and permits restart. A failed stop keeps the configuration so
 * feeds remain possible; do not assume the hardware is stopped on error.
 * The nRF54L15 SDK supports stopping; other backends may return -EPERM.
 */
int nrf54l15_watchdog_stop(struct nrf54l15_watchdog *watchdog);

#ifdef __cplusplus
}
#endif
#endif
