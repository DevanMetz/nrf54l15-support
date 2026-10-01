/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <nrf54l15/imu.h>

int nrf54l15_imu_init(struct nrf54l15_imu *imu, const struct device *device,
		     const struct nrf54l15_imu_config *config)
{
	if (imu == NULL) {
		return -EINVAL;
	}
	*imu = (struct nrf54l15_imu){.device = device};
	if (device == NULL || config == NULL || config->sample_hz == 0 ||
	    (config->accel_range_g != 2 && config->accel_range_g != 4 &&
	     config->accel_range_g != 8 && config->accel_range_g != 16) ||
	    (config->gyro_range_dps != 125 && config->gyro_range_dps != 250 &&
	     config->gyro_range_dps != 500 && config->gyro_range_dps != 1000 &&
	     config->gyro_range_dps != 2000)) {
		return -EINVAL;
	}
	if (!device_is_ready(device)) {
		return -ENODEV;
	}
	struct sensor_value range;
	struct sensor_value odr = {.val1 = config->sample_hz};
	int err;

	sensor_g_to_ms2(config->accel_range_g, &range);
	err = sensor_attr_set(device, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &range);
	if (err != 0) {
		return err;
	}
	sensor_degrees_to_rad(config->gyro_range_dps, &range);
	err = sensor_attr_set(device, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &range);
	if (err != 0) {
		return err;
	}
	err = sensor_attr_set(device, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (err != 0) {
		return err;
	}
	err = sensor_attr_set(device, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (err != 0) {
		return err;
	}
	imu->ready = true;
	return 0;
}

int nrf54l15_imu_read(struct nrf54l15_imu *imu, struct nrf54l15_imu_sample *sample)
{
	if (imu == NULL || sample == NULL) {
		return -EINVAL;
	}
	if (!imu->ready || imu->device == NULL) {
		return -ENODEV;
	}
	struct sensor_value accel[3], gyro[3];
	struct nrf54l15_imu_sample next;
	int err = sensor_sample_fetch(imu->device);

	if (err != 0) {
		return err;
	}
	next.timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());
	err = sensor_channel_get(imu->device, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (err != 0) {
		return err;
	}
	err = sensor_channel_get(imu->device, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (err != 0) {
		return err;
	}
	next.accel_mps2 = (struct nrf54l15_vec3){sensor_value_to_float(&accel[0]),
		sensor_value_to_float(&accel[1]), sensor_value_to_float(&accel[2])};
	next.gyro_rps = (struct nrf54l15_vec3){sensor_value_to_float(&gyro[0]),
		sensor_value_to_float(&gyro[1]), sensor_value_to_float(&gyro[2])};
	*sample = next;
	return 0;
}
