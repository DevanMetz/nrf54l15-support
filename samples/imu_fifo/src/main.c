/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <nrf54l15/lsm6ds3tr.h>
#include <nrf54l15/orientation.h>
#include <nrf54l15/telemetry.h>

#define SAMPLE_HZ 416
#define MAX_DRAIN 32
static const struct i2c_dt_spec imu_bus = I2C_DT_SPEC_GET(DT_ALIAS(imu0));

int main(void)
{
	struct nrf54l15_lsm6ds3tr sensor;
	const struct nrf54l15_imu_config config = {SAMPLE_HZ, 16, 2000};
	int err = nrf54l15_lsm6ds3tr_init(&sensor, &imu_bus, &config);
	if (!err) {
		err = nrf54l15_lsm6ds3tr_fifo_start(&sensor);
	}
	if (err) {
		printk("# IMU FIFO init failed: %d\n", err);
		return err;
	}
	printk("# Paired IMU FIFO: %d Hz, +/-16 g, +/-2000 dps, native sensor axes\n", SAMPLE_HZ);
	printk("# Fetch timestamps are host times, NOT sensor acquisition times\n");
	printk("# IMUFIFO,epoch,seq,fetch_us,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps,"
	       "roll_cdeg,pitch_cdeg,yaw_cdeg\n");
	struct nrf54l15_quat orientation = {1, 0, 0, 0};
	uint32_t epoch = 0, samples = 0, restarts = 0, printed_at = 0;
	unsigned largest_batch = 0;
	int64_t status_at = k_uptime_get();
	int64_t last_data_ms = status_at;
	while (true) {
		struct nrf54l15_lsm6ds3tr_fifo_sample latest;
		bool have_latest = false;
		unsigned drained = 0;
		/* Bound work per pass even when incoming data keeps filling FIFO. */
		for (; drained < MAX_DRAIN; ++drained) {
			struct nrf54l15_lsm6ds3tr_fifo_sample next;
			err = nrf54l15_lsm6ds3tr_fifo_read(&sensor, &next);
			if (err == -EAGAIN) {
				break;
			}
			if (err) {
				++restarts;
				++epoch;
				printk("# FIFO fault %d: discard backlog and begin epoch %u\n", err, epoch);
				err = nrf54l15_lsm6ds3tr_fifo_start(&sensor);
				if (err) {
					printk("# FIFO recovery failed: %d\n", err);
					return err;
				}
				/* Missing history invalidates orientation continuity. */
				orientation = (struct nrf54l15_quat){1, 0, 0, 0};
				last_data_ms = k_uptime_get();
				have_latest = false;
				break;
			}
			err = nrf54l15_madgwick_update(&orientation, &next.sample.gyro_rps,
				&next.sample.accel_mps2, 0.08f, 1.0f / SAMPLE_HZ);
			if (err) {
				printk("# Filter update failed: %d\n", err);
				return err;
			}
			++samples;
			last_data_ms = k_uptime_get();
			latest = next;
			have_latest = true;
		}
		if (drained > largest_batch) {
			largest_batch = drained;
		}
		/* Filter every pair; serialize only occasional snapshots. */
		if (have_latest && samples - printed_at >= 16) {
			struct nrf54l15_vec3 angles;
			err = nrf54l15_quat_to_euler(&orientation, &angles);
			if (err) {
				return err;
			}
			printed_at = samples;
			const float mg = 1000.0f / NRF54L15_GRAVITY_MPS2;
			const float mdps = NRF54L15_RAD_TO_DEG * 1000;
			printk("IMUFIFO,%u,%u,%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			       epoch, latest.sequence, (unsigned long long)latest.sample.timestamp_us,
			       nrf54l15_scale_i32(latest.sample.accel_mps2.x, mg),
			       nrf54l15_scale_i32(latest.sample.accel_mps2.y, mg),
			       nrf54l15_scale_i32(latest.sample.accel_mps2.z, mg),
			       nrf54l15_scale_i32(latest.sample.gyro_rps.x, mdps),
			       nrf54l15_scale_i32(latest.sample.gyro_rps.y, mdps),
			       nrf54l15_scale_i32(latest.sample.gyro_rps.z, mdps),
			       nrf54l15_scale_i32(angles.x, 100),
			       nrf54l15_scale_i32(angles.y, 100),
			       nrf54l15_scale_i32(angles.z, 100));
		}
		int64_t now_ms = k_uptime_get();
		if (now_ms - last_data_ms >= 1000) {
			printk("# FIFO timed out: no complete samples for 1 second\n");
			return -ETIMEDOUT;
		}
		if (now_ms - status_at >= 1000) {
			status_at = now_ms;
			printk("# FIFOSTATS,epoch=%u,samples=%u,restarts=%u,max_batch=%u\n",
			       epoch, samples, restarts, largest_batch);
		}
		/* A full batch may leave a backlog: continue draining immediately. */
		if (drained < MAX_DRAIN) { k_sleep(K_MSEC(10)); }
	}
}
