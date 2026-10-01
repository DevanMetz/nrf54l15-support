/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <nrf54l15/gyro_calibration.h>
#include <nrf54l15/lsm6ds3tr.h>
#include <nrf54l15/orientation.h>
#include <nrf54l15/telemetry.h>
#ifdef CONFIG_SAMPLE_CALIBRATION_PERSISTENCE
#include <nrf54l15/button.h>
#include <nrf54l15/gyro_bias.h>
#endif

static const struct i2c_dt_spec imu_bus = I2C_DT_SPEC_GET(DT_ALIAS(imu0));

static int calibrate(struct nrf54l15_lsm6ds3tr *sensor, struct nrf54l15_vec3 *bias)
{
	const struct nrf54l15_gyro_calibration_config config = {
		.samples_required = 208, /* Two seconds at 104 Hz. */
		.max_gyro_rps = 0.15f,
		.max_accel_error_mps2 = 1.0f,
		.max_gyro_stddev_rps = 0.015f,
		.max_accel_stddev_mps2 = 0.2f,
	};
	struct nrf54l15_gyro_calibration state;
	int err = nrf54l15_gyro_calibration_begin(&state, &config);
	if (err) {
		return err;
	}
	printk("# Hold the board STILL on a stable surface; calibration starts in 1 second\n");
	printk("# Motion restarts the 208-sample window; operation times out after 30 seconds\n");
	k_sleep(K_SECONDS(1));
	int64_t deadline_ms = k_uptime_get() + 30000;
	uint64_t previous_us = 0;
	unsigned motion_resets = 0, gap_resets = 0;
	while (k_uptime_get() < deadline_ms) {
		struct nrf54l15_imu_sample sample;
		err = nrf54l15_lsm6ds3tr_read(sensor, &sample);
		if (err && err != -EAGAIN) {
			return err;
		}
		if (err == 0) {
			/* Do not join calibration data across missed samples. */
			if (previous_us && sample.timestamp_us - previous_us > 20000) {
				(void)nrf54l15_gyro_calibration_begin(&state, &config);
				++gap_resets;
			}
			previous_us = sample.timestamp_us;
			err = nrf54l15_gyro_calibration_update(&state,
				&sample.accel_mps2, &sample.gyro_rps);
			if (err < 0) {
				return err;
			}
			if (err == NRF54L15_CALIBRATION_COMPLETE) {
				printk("# Calibration complete; motion_resets=%u, gap_resets=%u\n",
				       motion_resets, gap_resets);
				return nrf54l15_gyro_calibration_result(&state, bias);
			}
			if (err == NRF54L15_CALIBRATION_MOTION) {
				++motion_resets;
			}
		}
		k_sleep(K_MSEC(2));
	}
	printk("# Calibration timed out; motion_resets=%u, gap_resets=%u\n",
	       motion_resets, gap_resets);
	return -ETIMEDOUT;
}

#ifdef CONFIG_SAMPLE_CALIBRATION_PERSISTENCE
/* Example profile v1: LSM6DS3TR-C, native sensor axes, 104 Hz, 4 g, 250 dps.
 * Change this ID if an old calibration is no longer applicable.
 */
#define BIAS_PROFILE UINT32_C(0x4c530001)

static int acquire_bias(struct nrf54l15_lsm6ds3tr *sensor, struct nrf54l15_vec3 *bias)
{
	const struct gpio_dt_spec gpio = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
	struct nrf54l15_button button;
	int err = nrf54l15_button_init(&button, &gpio, 20);
	if (err) {
		return err;
	}
	bool replace = false;
	/* Sample for longer than the debounce interval; pressing during ordinary
	 * streaming is not a storage operation in this example.
	 */
	for (unsigned int i = 0; i < 7; ++i) {
		err = nrf54l15_button_poll(&button, (uint32_t)k_uptime_get());
		if (err < 0) {
			return err;
		}
		if (err == NRF54L15_BUTTON_PRESSED) {
			replace = true;
		}
		k_sleep(K_MSEC(5));
	}
	if (!replace) {
		err = nrf54l15_gyro_bias_load(BIAS_PROFILE, bias);
		if (!err) {
			printk("# Restored gyro bias from Settings\n");
			return 0;
		}
		if (err == -ENOENT) {
			printk("# No saved gyro bias was found\n");
		} else {
			printk("# Stored calibration could not be loaded: %d\n", err);
		}
		/* The SDK's Settings/ZMS name lookup can hide read failures as a
		 * missing key. Never infer permission to replace data from absence.
		 */
		printk("# To measure and save a bias, hold the user button while resetting\n");
		return err;
	}
	printk("# Calibration requested; release the button and place the board flat\n");
	err = calibrate(sensor, bias);
	if (err) {
		return err;
	}
	err = nrf54l15_gyro_bias_save(BIAS_PROFILE, bias);
	if (err) {
		printk("# Saving gyro bias failed: %d\n", err);
		return err;
	}
	printk("# Saved gyro bias to Settings\n");
	return 0;
}
#endif

int main(void)
{
	struct nrf54l15_lsm6ds3tr sensor;
	const struct nrf54l15_imu_config config = {104, 4, 250};
	int err = nrf54l15_lsm6ds3tr_init(&sensor, &imu_bus, &config);
	if (err) {
		printk("# IMU init failed: %d\n", err);
		return err;
	}
	struct nrf54l15_vec3 bias;
#ifdef CONFIG_SAMPLE_CALIBRATION_PERSISTENCE
	err = acquire_bias(&sensor, &bias);
#else
	err = calibrate(&sensor, &bias);
#endif
	if (err) {
		printk("# Calibration failed: %d; reset the board to try again\n", err);
		return err;
	}
	const float mdps = NRF54L15_RAD_TO_DEG * 1000;
	printk("# GBIAS,x_mdps,y_mdps,z_mdps\n");
#ifdef CONFIG_SAMPLE_CALIBRATION_PERSISTENCE
	printk("# Bias persists; hold the user button during reset to recalibrate\n");
#else
	printk("# Bias is in RAM only; reset recalibrates\n");
#endif
	printk("GBIAS,%d,%d,%d\n", nrf54l15_scale_i32(bias.x, mdps),
	       nrf54l15_scale_i32(bias.y, mdps), nrf54l15_scale_i32(bias.z, mdps));
	printk("# GCORR,seq,uptime_us,raw_gx_mdps,raw_gy_mdps,raw_gz_mdps,"
	       "gx_mdps,gy_mdps,gz_mdps,roll_cdeg,pitch_cdeg,yaw_cdeg\n");
	struct nrf54l15_quat orientation = {1, 0, 0, 0};
	uint64_t previous_us = 0;
	uint32_t sequence = 0;
	while (true) {
		struct nrf54l15_imu_sample sample;
		err = nrf54l15_lsm6ds3tr_read(&sensor, &sample);
		if (err == 0) {
			struct nrf54l15_vec3 corrected = {
				sample.gyro_rps.x - bias.x,
				sample.gyro_rps.y - bias.y,
				sample.gyro_rps.z - bias.z,
			};
			float dt = previous_us ? (sample.timestamp_us - previous_us) * 1.0e-6f : 1.0f / 104;
			previous_us = sample.timestamp_us;
			/* A long gap has no known rotation history; retain the previous pose. */
			if (dt > 0 && dt <= 0.2f) {
				err = nrf54l15_madgwick_update(&orientation, &corrected,
					&sample.accel_mps2, 0.08f, dt);
				if (err) {
					printk("# Filter update failed: %d\n", err);
					return err;
				}
			}
			if (sequence % 16 == 0) {
				struct nrf54l15_vec3 angles;
				err = nrf54l15_quat_to_euler(&orientation, &angles);
				if (err) {
					return err;
				}
				printk("GCORR,%u,%llu,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", sequence,
				       (unsigned long long)sample.timestamp_us,
				       nrf54l15_scale_i32(sample.gyro_rps.x, mdps),
				       nrf54l15_scale_i32(sample.gyro_rps.y, mdps),
				       nrf54l15_scale_i32(sample.gyro_rps.z, mdps),
				       nrf54l15_scale_i32(corrected.x, mdps),
				       nrf54l15_scale_i32(corrected.y, mdps),
				       nrf54l15_scale_i32(corrected.z, mdps),
				       nrf54l15_scale_i32(angles.x, 100),
				       nrf54l15_scale_i32(angles.y, 100),
				       nrf54l15_scale_i32(angles.z, 100));
			}
			++sequence;
		} else if (err != -EAGAIN) {
			printk("# IMU read failed: %d\n", err);
			return err;
		}
		k_sleep(K_MSEC(2));
	}
}
