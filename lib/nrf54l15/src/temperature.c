/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <limits.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <nrf54l15/temperature.h>
#if defined(CONFIG_TEMP_NRF5_MPSL)
#include <multithreading_lock.h>
#endif

int nrf54l15_die_temperature_read(const struct device *device,
	struct nrf54l15_temperature_sample *sample)
{
	if (!device || !sample) { return -EINVAL; }
	if (!device_is_ready(device)) { return -ENODEV; }
	int err;
#if defined(CONFIG_TEMP_NRF5_MPSL)
	/* NCS's MPSL sensor backend calls mpsl_temperature_get without taking the
	 * lock used by mpsl_low_priority_process and the SoftDevice controller.
	 */
	err = multithreading_lock_acquire(K_NO_WAIT);
	if (err) { return err; }
#endif
	struct sensor_value value;
	err = sensor_sample_fetch_chan(device, SENSOR_CHAN_DIE_TEMP);
	if (!err) { err = sensor_channel_get(device, SENSOR_CHAN_DIE_TEMP, &value); }
#if defined(CONFIG_TEMP_NRF5_MPSL)
	multithreading_lock_release();
#endif
	if (err) { return err; }
	int64_t micro = (int64_t)value.val1 * 1000000 + value.val2;
	int64_t milli = (micro >= 0 ? micro + 500 : micro - 500) / 1000;
	if (milli < INT32_MIN || milli > INT32_MAX) { return -ERANGE; }
	*sample = (struct nrf54l15_temperature_sample){
		.timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks()),
		.millicelsius = (int32_t)milli,
	};
	return 0;
}
