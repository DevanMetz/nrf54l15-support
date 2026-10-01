/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <nrf54l15/lsm6ds3tr.h>
#include <nrf54l15/telemetry.h>

static const struct i2c_dt_spec imu_bus = I2C_DT_SPEC_GET(DT_ALIAS(imu0));

int main(void)
{
	struct nrf54l15_lsm6ds3tr sensor;
	const struct nrf54l15_imu_config config = {104, 16, 2000};
	uint32_t sequence = 0;
	int err = nrf54l15_lsm6ds3tr_init(&sensor, &imu_bus, &config);
	if (err) {
		printk("# IMU init failed: %d\n", err);
		return err;
	}
	printk("# Direct LSM6DS3TR-C I2C driver: 104 Hz, +/-16 g, +/-2000 dps\n");
	printk("# IMU,seq,uptime_us,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps\n");
	while (true) {
		struct nrf54l15_imu_sample sample;
		err = nrf54l15_lsm6ds3tr_read(&sensor, &sample);
		if (err == 0) {
			if (sequence % 16 == 0) {
				printk("IMU,%u,%llu,%d,%d,%d,%d,%d,%d\n", sequence,
				       (unsigned long long)sample.timestamp_us,
				       nrf54l15_scale_i32(sample.accel_mps2.x, 1000 / NRF54L15_GRAVITY_MPS2),
				       nrf54l15_scale_i32(sample.accel_mps2.y, 1000 / NRF54L15_GRAVITY_MPS2),
				       nrf54l15_scale_i32(sample.accel_mps2.z, 1000 / NRF54L15_GRAVITY_MPS2),
				       nrf54l15_scale_i32(sample.gyro_rps.x, NRF54L15_RAD_TO_DEG * 1000),
				       nrf54l15_scale_i32(sample.gyro_rps.y, NRF54L15_RAD_TO_DEG * 1000),
				       nrf54l15_scale_i32(sample.gyro_rps.z, NRF54L15_RAD_TO_DEG * 1000));
			}
			++sequence;
		} else if (err != -EAGAIN) {
			printk("# IMU read failed: %d\n", err);
			k_sleep(K_MSEC(100));
		}
		k_sleep(K_MSEC(2));
	}
}
