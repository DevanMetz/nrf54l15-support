/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <nrf54l15/lsm6ds3tr.h>

static const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(DT_ALIAS(imu0));

static int capture(struct nrf54l15_lsm6ds3tr *sensor, uint64_t *last_fetch_us)
{
	int64_t began_ms = k_uptime_get();
	unsigned int samples = 0;
	while (samples < 104) {
		struct nrf54l15_imu_sample sample;
		int err = nrf54l15_lsm6ds3tr_read(sensor, &sample);
		if (!err) {
			++samples;
			*last_fetch_us = sample.timestamp_us;
		} else if (err != -EAGAIN) {
			return err;
		}
		if (k_uptime_get() - began_ms >= 3000) { return -ETIMEDOUT; }
		k_sleep(K_MSEC(2));
	}
	return 0;
}

int main(void)
{
	struct nrf54l15_lsm6ds3tr sensor;
	const struct nrf54l15_imu_config config = {104, 4, 250};
	int err = nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config);
	if (err) {
		printk("# IMU power-cycle init failed: %d\n", err);
		return 0;
	}
	printk("# IMU power cycle: capture 104 samples at 104 Hz, then suspend for 2 s\n");
	printk("# Register power-down only; shared supply and MCU remain powered\n");
	for (uint32_t epoch = 0; !err; ++epoch) {
		uint64_t fetched_us = 0;
		err = capture(&sensor, &fetched_us);
		if (err) { break; }
		printk("IMUPOWER,%u,AWAKE,samples=104,last_fetch_us=%llu\n",
			epoch, (unsigned long long)fetched_us);
		err = nrf54l15_lsm6ds3tr_suspend(&sensor);
		if (err) { break; }
		struct nrf54l15_imu_sample blocked;
		if (nrf54l15_lsm6ds3tr_read(&sensor, &blocked) != -EACCES) {
			err = -EIO;
			break;
		}
		printk("IMUPOWER,%u,SUSPENDED,uptime_ms=%lld\n", epoch, (long long)k_uptime_get());
		k_sleep(K_MSEC(2000));
		err = nrf54l15_lsm6ds3tr_resume(&sensor);
		if (!err) {
			printk("IMUPOWER,%u,RESUMED,uptime_ms=%lld\n", epoch, (long long)k_uptime_get());
		}
	}
	/* One explicit cleanup attempt also handles a partially applied transition. */
	int suspend_err = nrf54l15_lsm6ds3tr_suspend(&sensor);
	printk("# IMU power cycle ended: error=%d suspend=%d\n", err, suspend_err);
	return 0;
}
