/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <nrf54l15/watchdog.h>

int nrf54l15_watchdog_start(struct nrf54l15_watchdog *watchdog,
	const struct device *dev, uint32_t timeout_ms, uint8_t channels, uint8_t options)
{
	if (watchdog == NULL || dev == NULL) {
		return -EINVAL;
	}
	if (watchdog->state == NRF54L15_WATCHDOG_RUNNING) {
		return -EALREADY;
	}
	if (watchdog->state != NRF54L15_WATCHDOG_IDLE) {
		return -EIO;
	}
	if (timeout_ms == 0 || timeout_ms > NRF54L15_WATCHDOG_MAX_TIMEOUT_MS ||
	    channels == 0 || channels > NRF54L15_WATCHDOG_MAX_CHANNELS) {
		return -EINVAL;
	}
	if ((options & ~(WDT_OPT_PAUSE_IN_SLEEP | WDT_OPT_PAUSE_HALTED_BY_DBG)) != 0) {
		return -ENOTSUP;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	watchdog->dev = dev;
	watchdog->channel_count = 0;
	watchdog->state = NRF54L15_WATCHDOG_FAULTED;
	const struct wdt_timeout_cfg config = {
		.window = {.min = 0, .max = timeout_ms},
		.flags = WDT_FLAG_RESET_SOC,
		.callback = NULL,
	};
	for (uint8_t slot = 0; slot < channels; ++slot) {
		int channel_id = wdt_install_timeout(dev, &config);
		if (channel_id < 0) {
			return channel_id;
		}
		watchdog->channel_ids[slot] = channel_id;
		++watchdog->channel_count;
	}
	int err = wdt_setup(dev, options);
	if (!err) {
		watchdog->state = NRF54L15_WATCHDOG_RUNNING;
	}
	return err;
}

int nrf54l15_watchdog_feed(const struct nrf54l15_watchdog *watchdog, uint8_t slot)
{
	if (watchdog == NULL) {
		return -EINVAL;
	}
	if (watchdog->state == NRF54L15_WATCHDOG_FAULTED) {
		return -EIO;
	}
	if (watchdog->state != NRF54L15_WATCHDOG_RUNNING) {
		return -ENODEV;
	}
	if (slot >= watchdog->channel_count) {
		return -EINVAL;
	}
	return wdt_feed(watchdog->dev, watchdog->channel_ids[slot]);
}

int nrf54l15_watchdog_stop(struct nrf54l15_watchdog *watchdog)
{
	if (watchdog == NULL) {
		return -EINVAL;
	}
	if (watchdog->state == NRF54L15_WATCHDOG_FAULTED) {
		return -EIO;
	}
	if (watchdog->state != NRF54L15_WATCHDOG_RUNNING) {
		return -ENODEV;
	}
	int err = wdt_disable(watchdog->dev);
	if (!err) {
		*watchdog = (struct nrf54l15_watchdog){0};
	}
	return err;
}
