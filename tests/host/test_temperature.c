/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <nrf54l15/temperature.h>
#if defined(CONFIG_TEMP_NRF5_MPSL)
#include <multithreading_lock.h>
#endif

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static struct device sensor = {.ready = true};
static struct sensor_value measured;
static int fetch_result, get_result, fetches, gets, clocks;
static int64_t ticks;
#if defined(CONFIG_TEMP_NRF5_MPSL)
static int lock_result, locks, unlocks;
static bool locked;

int multithreading_lock_acquire(uint32_t timeout)
{
	CHECK(timeout == K_NO_WAIT && !locked);
	++locks;
	if (!lock_result) { locked = true; }
	return lock_result;
}

void multithreading_lock_release(void)
{
	CHECK(locked);
	locked = false;
	++unlocks;
}
#endif

int64_t k_uptime_ticks(void) { ++clocks; return ticks; }

int sensor_sample_fetch_chan(const struct device *device, enum sensor_channel channel)
{
	CHECK(device == &sensor && channel == SENSOR_CHAN_DIE_TEMP);
#if defined(CONFIG_TEMP_NRF5_MPSL)
	CHECK(locked);
#endif
	++fetches;
	ticks += 17;
	return fetch_result;
}

int sensor_channel_get(const struct device *device, enum sensor_channel channel,
	struct sensor_value *value)
{
	CHECK(device == &sensor && channel == SENSOR_CHAN_DIE_TEMP);
#if defined(CONFIG_TEMP_NRF5_MPSL)
	CHECK(locked);
#endif
	++gets;
	ticks += 3;
	*value = measured; /* Even a failed backend can overwrite its argument. */
	return get_result;
}

static void reset(void)
{
	sensor.ready = true;
	measured = (struct sensor_value){25, 250000};
	fetch_result = get_result = fetches = gets = clocks = 0;
	ticks = 10000000000LL;
#if defined(CONFIG_TEMP_NRF5_MPSL)
	CHECK(!locked);
	lock_result = locks = unlocks = 0;
#endif
}

static void conversions(void)
{
	const struct { int32_t degrees, micro, expected; } cases[] = {
		{25, 250000, 25250}, {0, 0, 0}, {-40, 0, -40000},
		{-1, -750000, -1750}, {0, -250000, -250}, {125, 750000, 125750},
		{0, 499, 0}, {0, -499, 0}, {0, 500, 1}, {0, -500, -1},
		{24, 937500, 24938}, {-24, -937500, -24938},
		/* Evaluate val1 + val2 / 1e6, including opposite-sign fractions. */
		{1, -500, 1000}, {-1, 500, -1000},
		{2147483, 647499, INT32_MAX}, {-2147483, -648499, INT32_MIN},
	};
	for (size_t i = 0; i < ARRAY_SIZE(cases); ++i) {
		reset();
		measured = (struct sensor_value){cases[i].degrees, cases[i].micro};
		struct nrf54l15_temperature_sample sample;
		CHECK(nrf54l15_die_temperature_read(&sensor, &sample) == 0);
		CHECK(sample.millicelsius == cases[i].expected);
		CHECK(sample.timestamp_us == 10000000020ULL);
		CHECK(fetches == 1 && gets == 1 && clocks == 1);
#if defined(CONFIG_TEMP_NRF5_MPSL)
		CHECK(locks == 1 && unlocks == 1 && !locked);
#endif
	}
	/* Exercise every quarter-degree in and around the device's operating range. */
	for (int quarter = -400; quarter <= 800; ++quarter) {
		reset();
		measured = (struct sensor_value){quarter / 4, (quarter % 4) * 250000};
		struct nrf54l15_temperature_sample sample;
		CHECK(nrf54l15_die_temperature_read(&sensor, &sample) == 0);
		CHECK(sample.millicelsius == quarter * 250);
	}
}

static void failures_preserve_output(void)
{
	reset();
	struct nrf54l15_temperature_sample sample;
	memset(&sample, 0x63, sizeof(sample));
	unsigned char original[sizeof(sample)];
	memcpy(original, &sample, sizeof(sample));
	CHECK(nrf54l15_die_temperature_read(NULL, &sample) == -EINVAL);
	CHECK(nrf54l15_die_temperature_read(&sensor, NULL) == -EINVAL);
	sensor.ready = false;
	CHECK(nrf54l15_die_temperature_read(&sensor, &sample) == -ENODEV);
	CHECK(!fetches && !gets && !clocks);
#if defined(CONFIG_TEMP_NRF5_MPSL)
	CHECK(!locks && !unlocks);
#endif
	sensor.ready = true;
	const int failures[] = {-EIO, -EAGAIN, -ENOTSUP, -EBUSY};
	for (size_t i = 0; i < ARRAY_SIZE(failures); ++i) {
		for (int stage = 0; stage < 2; ++stage) {
			reset();
			if (stage) { get_result = failures[i]; } else { fetch_result = failures[i]; }
			CHECK(nrf54l15_die_temperature_read(&sensor, &sample) == failures[i]);
			CHECK(!memcmp(original, &sample, sizeof(sample)));
			CHECK(fetches == 1 && gets == stage && clocks == 0);
#if defined(CONFIG_TEMP_NRF5_MPSL)
			CHECK(locks == 1 && unlocks == 1 && !locked);
#endif
		}
	}
	const struct sensor_value outside[] = {
		{2147483, 647500}, {-2147483, -648500},
		{INT32_MAX, INT32_MAX}, {INT32_MIN, INT32_MIN},
	};
	for (size_t i = 0; i < ARRAY_SIZE(outside); ++i) {
		reset();
		measured = outside[i];
		CHECK(nrf54l15_die_temperature_read(&sensor, &sample) == -ERANGE);
		CHECK(!memcmp(original, &sample, sizeof(sample)) && !clocks);
#if defined(CONFIG_TEMP_NRF5_MPSL)
		CHECK(locks == 1 && unlocks == 1 && !locked);
#endif
	}
#if defined(CONFIG_TEMP_NRF5_MPSL)
	reset();
	lock_result = -EBUSY;
	CHECK(nrf54l15_die_temperature_read(&sensor, &sample) == -EBUSY);
	CHECK(!memcmp(original, &sample, sizeof(sample)));
	CHECK(!fetches && !gets && !clocks && locks == 1 && !unlocks && !locked);
	lock_result = 0;
	CHECK(nrf54l15_die_temperature_read(&sensor, &sample) == 0);
	CHECK(fetches == 1 && gets == 1 && locks == 2 && unlocks == 1 && !locked);
#endif
}

int main(void)
{
	conversions();
	failures_preserve_output();
	puts("PASS: die temperature conversion, fetch/get failures and output preservation");
	return 0;
}
