/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_GYRO_CALIBRATION_H
#define NRF54L15_SUPPORT_GYRO_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>
#include <nrf54l15/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_gyro_calibration_config {
	uint16_t samples_required; /* At least 2, uniformly spaced while held still. */
	float max_gyro_rps; /* Limit on the uncorrected angular-rate magnitude. */
	float max_accel_error_mps2; /* Allowed difference from standard gravity. */
	float max_gyro_stddev_rps; /* Limit on combined XYZ sample standard deviation. */
	float max_accel_stddev_mps2;
};

struct nrf54l15_gyro_calibration {
	struct nrf54l15_gyro_calibration_config config;
	struct nrf54l15_vec3 gyro_mean;
	struct nrf54l15_vec3 accel_mean;
	float gyro_m2;
	float accel_m2;
	uint16_t samples;
	bool complete;
};

enum nrf54l15_calibration_status {
	NRF54L15_CALIBRATION_COLLECTING = 0,
	NRF54L15_CALIBRATION_COMPLETE = 1,
	NRF54L15_CALIBRATION_MOTION = 2,
};

/* Explicit stationary calibration, not an automatic drift observer. A slow
 * constant yaw rotation cannot be distinguished from bias using a six-axis
 * IMU alone. The operator must hold the device still during this operation.
 * begin resets the window, including a previously completed result. Call it
 * again after missing/error samples to avoid joining discontinuous windows.
 * Failed begin leaves the previous state intact. No allocation or OS API.
 */
int nrf54l15_gyro_calibration_begin(struct nrf54l15_gyro_calibration *state,
	const struct nrf54l15_gyro_calibration_config *config);

/* Returns a status above, or negative errno. Motion/invalid data discard the
 * pending window. Completion freezes the result until begin is called again.
 */
int nrf54l15_gyro_calibration_update(struct nrf54l15_gyro_calibration *state,
	const struct nrf54l15_vec3 *accel, const struct nrf54l15_vec3 *gyro);

/* Copy the bias (radians/s) after completion; otherwise -EAGAIN. Caller owns
 * the committed bias and subtracts it from subsequent angular-rate samples.
 * Output is unchanged on failure. Keep raw and corrected data distinguishable.
 */
int nrf54l15_gyro_calibration_result(const struct nrf54l15_gyro_calibration *state,
	struct nrf54l15_vec3 *bias_rps);

#ifdef __cplusplus
}
#endif
#endif
