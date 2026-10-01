/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <nrf54l15/watchdog.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static struct device dev = {.ready = true};
static unsigned int install_calls, setup_calls, feed_calls, stop_calls, allocated;
static unsigned int fail_install, pending_feeds;
static uint32_t timeout_ms;
static uint8_t received_options;
static int setup_error, feed_error, stop_error;
static int last_channel;
static bool armed, expired;
static uint64_t now_ms, deadline_ms;

static void reset_backend(void)
{
	install_calls = setup_calls = feed_calls = stop_calls = allocated = 0;
	fail_install = pending_feeds = 0;
	setup_error = feed_error = stop_error = 0;
	armed = expired = false;
	dev.ready = true;
	now_ms = deadline_ms = 0;
}

int wdt_install_timeout(const struct device *device, const struct wdt_timeout_cfg *config)
{
	CHECK(device == &dev && !armed);
	CHECK(config->window.min == 0 && config->window.max > 0);
	CHECK(config->flags == WDT_FLAG_RESET_SOC && config->callback == NULL);
	++install_calls;
	if (install_calls == fail_install) {
		return -ENOMEM;
	}
	if (allocated) {
		CHECK(config->window.max == timeout_ms);
	}
	timeout_ms = config->window.max;
	return (int)allocated++;
}

int wdt_setup(const struct device *device, uint8_t options)
{
	CHECK(device == &dev && allocated > 0 && !armed);
	++setup_calls;
	received_options = options;
	if (setup_error) {
		return setup_error;
	}
	armed = true;
	pending_feeds = 0;
	deadline_ms = now_ms + timeout_ms;
	return 0;
}

int wdt_feed(const struct device *device, int channel)
{
	CHECK(device == &dev && armed && !expired);
	CHECK(channel >= 0 && (unsigned int)channel < allocated);
	last_channel = channel;
	++feed_calls;
	if (feed_error) {
		return feed_error;
	}
	pending_feeds |= 1U << channel;
	if (pending_feeds == (1U << allocated) - 1U) {
		deadline_ms = now_ms + timeout_ms;
		pending_feeds = 0;
	}
	return 0;
}

int wdt_disable(const struct device *device)
{
	CHECK(device == &dev && armed);
	++stop_calls;
	if (stop_error) {
		return stop_error;
	}
	armed = false;
	allocated = 0;
	return 0;
}

/* Logical model of the datasheet's all-enabled-channels reload rule. This is
 * not an emulation of oscillator timing, sleep/debug pausing or hardware reset.
 */
static void advance(uint64_t milliseconds)
{
	now_ms += milliseconds;
	if (armed && now_ms >= deadline_ms) {
		expired = true;
	}
}

static void test_channels_and_lifecycle(void)
{
	for (uint8_t channels = 1; channels <= NRF54L15_WATCHDOG_MAX_CHANNELS; ++channels) {
		for (uint8_t options = 0; options <= 3; ++options) {
			reset_backend();
			struct nrf54l15_watchdog watchdog = {0};
			CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 2000, channels, options) == 0);
			CHECK(watchdog.state == NRF54L15_WATCHDOG_RUNNING && armed);
			CHECK(watchdog.channel_count == channels && install_calls == channels);
			CHECK(setup_calls == 1 && received_options == options && feed_calls == 0);
			for (uint8_t slot = 0; slot < channels; ++slot) {
				CHECK(nrf54l15_watchdog_feed(&watchdog, slot) == 0 && last_channel == slot);
			}
			CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 1, 0) == -EALREADY);
			CHECK(install_calls == channels && setup_calls == 1);
			unsigned int before = feed_calls;
			CHECK(nrf54l15_watchdog_feed(&watchdog, channels) == -EINVAL && feed_calls == before);
			CHECK(nrf54l15_watchdog_stop(&watchdog) == 0 && !armed);
			CHECK(watchdog.state == NRF54L15_WATCHDOG_IDLE && watchdog.dev == NULL);
			CHECK(watchdog.channel_count == 0);
			CHECK(nrf54l15_watchdog_feed(&watchdog, 0) == -ENODEV);
			CHECK(nrf54l15_watchdog_stop(&watchdog) == -ENODEV && stop_calls == 1);
			CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1, 1, 0) == 0);
			CHECK(timeout_ms == 1);
			CHECK(nrf54l15_watchdog_stop(&watchdog) == 0);
		}
	}
}

static void test_missing_worker(void)
{
	reset_backend();
	struct nrf54l15_watchdog watchdog = {0};
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 2, 0) == 0);
	CHECK(nrf54l15_watchdog_feed(&watchdog, 0) == 0);
	CHECK(nrf54l15_watchdog_feed(&watchdog, 1) == 0);
	advance(100);
	CHECK(nrf54l15_watchdog_feed(&watchdog, 1) == 0);
	advance(800);
	CHECK(nrf54l15_watchdog_feed(&watchdog, 0) == 0 && deadline_ms == 1900);
	/* Slot 1 has stopped. Repeated progress from slot 0 cannot hide it. */
	for (unsigned int i = 0; i < 9; ++i) {
		advance(100);
		CHECK(!expired && nrf54l15_watchdog_feed(&watchdog, 0) == 0);
	}
	CHECK(now_ms == 1800 && deadline_ms == 1900);
	advance(100);
	CHECK(expired);
	/* Staggered feeds mean this is 1800 ms after slot 1's last feed, not
	 * an independent 1000 ms deadline from that worker's last progress.
	 */
}

static void test_invalid_and_failure(void)
{
	reset_backend();
	struct nrf54l15_watchdog watchdog = {0};
	CHECK(nrf54l15_watchdog_start(NULL, &dev, 1000, 1, 0) == -EINVAL);
	CHECK(nrf54l15_watchdog_start(&watchdog, NULL, 1000, 1, 0) == -EINVAL);
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 0, 1, 0) == -EINVAL);
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, UINT32_MAX, 1, 0) == -EINVAL);
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, NRF54L15_WATCHDOG_MAX_TIMEOUT_MS + 1, 1, 0) == -EINVAL);
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 0, 0) == -EINVAL);
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 9, 0) == -EINVAL);
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 1, 4) == -ENOTSUP);
	dev.ready = false;
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 1, 0) == -ENODEV);
	dev.ready = true;
	CHECK(install_calls == 0 && setup_calls == 0 && watchdog.state == NRF54L15_WATCHDOG_IDLE);
	CHECK(nrf54l15_watchdog_feed(NULL, 0) == -EINVAL);
	CHECK(nrf54l15_watchdog_stop(NULL) == -EINVAL);
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, NRF54L15_WATCHDOG_MAX_TIMEOUT_MS, 1, 0) == 0);
	CHECK(timeout_ms == NRF54L15_WATCHDOG_MAX_TIMEOUT_MS);
	feed_error = -EAGAIN;
	CHECK(nrf54l15_watchdog_feed(&watchdog, 0) == -EAGAIN && pending_feeds == 0);
	feed_error = 0;
	stop_error = -EPERM;
	CHECK(nrf54l15_watchdog_stop(&watchdog) == -EPERM && armed);
	CHECK(watchdog.state == NRF54L15_WATCHDOG_RUNNING && watchdog.channel_count == 1);
	CHECK(nrf54l15_watchdog_feed(&watchdog, 0) == 0);
	stop_error = 0;
	CHECK(nrf54l15_watchdog_stop(&watchdog) == 0);
	for (unsigned int fail = 1; fail <= 8; ++fail) {
		reset_backend();
		watchdog = (struct nrf54l15_watchdog){0};
		fail_install = fail;
		CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 8, 0) == -ENOMEM);
		CHECK(watchdog.state == NRF54L15_WATCHDOG_FAULTED);
		CHECK(watchdog.channel_count == fail - 1 && setup_calls == 0 && !armed);
		CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 8, 0) == -EIO);
		CHECK(nrf54l15_watchdog_feed(&watchdog, 0) == -EIO);
		CHECK(nrf54l15_watchdog_stop(&watchdog) == -EIO);
		CHECK(install_calls == fail && feed_calls == 0 && stop_calls == 0);
	}
	reset_backend();
	watchdog = (struct nrf54l15_watchdog){0};
	setup_error = -EIO;
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 2, 0) == -EIO);
	CHECK(watchdog.state == NRF54L15_WATCHDOG_FAULTED && watchdog.channel_count == 2);
	CHECK(nrf54l15_watchdog_start(&watchdog, &dev, 1000, 2, 0) == -EIO);
	CHECK(install_calls == 2 && setup_calls == 1);
}

int main(void)
{
	test_channels_and_lifecycle();
	test_missing_worker();
	test_invalid_and_failure();
	puts("PASS: watchdog channels, lifecycle, partial startup, feed/stop errors and missing-worker model");
	return 0;
}
