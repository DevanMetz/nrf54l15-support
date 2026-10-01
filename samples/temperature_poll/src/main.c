/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <nrf54l15/lsm6ds3tr.h>
#include <nrf54l15/temperature.h>

static const struct device *const die = DEVICE_DT_GET(DT_NODELABEL(temp));
static const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(DT_ALIAS(imu0));

static int read_pair(struct nrf54l15_lsm6ds3tr *imu,
	struct nrf54l15_temperature_sample *die_sample,
	struct nrf54l15_temperature_sample *imu_sample)
{
	bool have_die = false, have_imu = false;
	/* Bound retries for no data / lock contention. This cannot impose a
	 * deadline on an individual blocking SDK temperature or I2C operation.
	 */
	for (unsigned int attempt = 0; attempt < 200; ++attempt) {
		if (!have_die) {
			int err = nrf54l15_die_temperature_read(die, die_sample);
			if (err && err != -EAGAIN && err != -EBUSY) { return err; }
			have_die = !err;
		}
		if (!have_imu) {
			int err = nrf54l15_lsm6ds3tr_temperature_read(imu, imu_sample);
			if (err && err != -EAGAIN) { return err; }
			have_imu = !err;
		}
		if (have_die && have_imu) { return 0; }
		k_sleep(K_MSEC(5));
	}
	return -ETIMEDOUT;
}

int main(void)
{
	struct nrf54l15_lsm6ds3tr imu;
	const struct nrf54l15_imu_config config = {52, 2, 250};
	if (!device_is_ready(die)) {
		printk("# SoC temperature device is not ready\n");
		return 0;
	}
	int err = nrf54l15_lsm6ds3tr_init(&imu, &bus, &config);
	if (err) {
		printk("# IMU temperature init failed: %d\n", err);
		return 0;
	}
#if defined(CONFIG_TEMP_NRF5_MPSL)
	printk("# MCU temperature backend: MPSL, shared lock enabled\n");
#else
	printk("# MCU temperature backend: Zephyr nRF TEMP\n");
#endif
	printk("# Die temperatures; IMU has uncalibrated offset. Fetches are not synchronized.\n");
	printk("# format: TEMP,die_fetch_us,die_mC,imu_fetch_us,imu_mC\n");
	for (;;) {
		struct nrf54l15_temperature_sample die_sample, imu_sample;
		err = read_pair(&imu, &die_sample, &imu_sample);
		if (err) { break; }
		printk("TEMP,%llu,%d,%llu,%d\n", (unsigned long long)die_sample.timestamp_us,
			die_sample.millicelsius, (unsigned long long)imu_sample.timestamp_us,
			imu_sample.millicelsius);
		k_sleep(K_MSEC(1000));
	}
	int suspend_err = nrf54l15_lsm6ds3tr_suspend(&imu);
	printk("# Temperature sample ended: error=%d IMU suspend=%d\n", err, suspend_err);
	return 0;
}
