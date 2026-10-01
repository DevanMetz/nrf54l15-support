/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_IMU_H
#define NRF54L15_SUPPORT_IMU_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <nrf54l15/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* LSM6DSL-compatible XIAO Sense IMU, using the upstream Zephyr driver.
 * The application owns power/devicetree setup and all calls must be serialized.
 * Use runtime FS/ODR Kconfig values (0) for CONFIG_LSM6DSL_{ACCEL,GYRO}_{FS,ODR}.
 */
struct nrf54l15_imu_config {
	uint16_t sample_hz;
	uint16_t accel_range_g; /* 2, 4, 8, 16 */
	uint16_t gyro_range_dps; /* 125, 250, 500, 1000, 2000 */
};

struct nrf54l15_imu {
	const struct device *device;
	bool ready;
};

struct nrf54l15_imu_sample {
	uint64_t timestamp_us; /* Host uptime at fetch completion, not a hardware timestamp. */
	struct nrf54l15_vec3 accel_mps2;
	struct nrf54l15_vec3 gyro_rps;
};

/* Configure both ranges and rates. On failure, ready=false; hardware may be
 * partially configured, so retry initialization before attempting to read.
 * Wait for sensor settling in the application after success.
 */
int nrf54l15_imu_init(struct nrf54l15_imu *imu, const struct device *device,
		     const struct nrf54l15_imu_config *config);
/* Fetches both channels; output remains unchanged on error. No axis rotation. */
int nrf54l15_imu_read(struct nrf54l15_imu *imu, struct nrf54l15_imu_sample *sample);

#ifdef __cplusplus
}
#endif
#endif
