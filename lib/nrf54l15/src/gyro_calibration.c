/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <nrf54l15/gyro_calibration.h>

static bool positive_finite_square(float value)
{
	return value > 0 && isfinite(value * value);
}

static bool config_valid(const struct nrf54l15_gyro_calibration_config *config)
{
	return config != NULL && config->samples_required >= 2 &&
		positive_finite_square(config->max_gyro_rps) &&
		positive_finite_square(config->max_accel_error_mps2) &&
		config->max_accel_error_mps2 < NRF54L15_GRAVITY_MPS2 &&
		positive_finite_square(config->max_gyro_stddev_rps) &&
		positive_finite_square(config->max_accel_stddev_mps2);
}

static void reset_window(struct nrf54l15_gyro_calibration *state)
{
	struct nrf54l15_gyro_calibration_config config = state->config;
	*state = (struct nrf54l15_gyro_calibration){.config = config};
}

int nrf54l15_gyro_calibration_begin(struct nrf54l15_gyro_calibration *state,
	const struct nrf54l15_gyro_calibration_config *config)
{
	if (state == NULL || !config_valid(config)) {
		return -EINVAL;
	}
	/* Copy first: restarting with &state->config is supported. */
	struct nrf54l15_gyro_calibration_config saved = *config;
	*state = (struct nrf54l15_gyro_calibration){.config = saved};
	return 0;
}

static bool finite_vec(const struct nrf54l15_vec3 *v)
{
	return isfinite(v->x) && isfinite(v->y) && isfinite(v->z);
}

static float norm2(const struct nrf54l15_vec3 *v)
{
	return v->x * v->x + v->y * v->y + v->z * v->z;
}

static float accumulate(struct nrf54l15_vec3 *mean,
			const struct nrf54l15_vec3 *value, float count)
{
	/* Welford's update avoids cancellation in E[x^2] - E[x]^2. */
	struct nrf54l15_vec3 delta = {value->x - mean->x, value->y - mean->y, value->z - mean->z};
	mean->x += delta.x / count;
	mean->y += delta.y / count;
	mean->z += delta.z / count;
	return delta.x * (value->x - mean->x) + delta.y * (value->y - mean->y) +
	       delta.z * (value->z - mean->z);
}

int nrf54l15_gyro_calibration_update(struct nrf54l15_gyro_calibration *state,
	const struct nrf54l15_vec3 *accel, const struct nrf54l15_vec3 *gyro)
{
	if (state == NULL || !config_valid(&state->config) || accel == NULL || gyro == NULL) {
		return -EINVAL;
	}
	if (state->complete) {
		return NRF54L15_CALIBRATION_COMPLETE;
	}
	if (!finite_vec(accel) || !finite_vec(gyro)) {
		reset_window(state);
		return -EINVAL;
	}
	float accel_norm2 = norm2(accel), gyro_norm2 = norm2(gyro);
	if (!isfinite(accel_norm2) || !isfinite(gyro_norm2)) {
		reset_window(state);
		return -ERANGE;
	}
	if (fabsf(sqrtf(accel_norm2) - NRF54L15_GRAVITY_MPS2) > state->config.max_accel_error_mps2 ||
	    gyro_norm2 > state->config.max_gyro_rps * state->config.max_gyro_rps) {
		reset_window(state);
		return NRF54L15_CALIBRATION_MOTION;
	}
	++state->samples;
	state->gyro_m2 += accumulate(&state->gyro_mean, gyro, state->samples);
	state->accel_m2 += accumulate(&state->accel_mean, accel, state->samples);
	if (!finite_vec(&state->gyro_mean) || !finite_vec(&state->accel_mean) ||
	    !isfinite(state->gyro_m2) || !isfinite(state->accel_m2)) {
		reset_window(state);
		return -ERANGE;
	}
	if (state->samples < state->config.samples_required) {
		return NRF54L15_CALIBRATION_COLLECTING;
	}
	float degrees_of_freedom = state->samples - 1;
	if (state->gyro_m2 / degrees_of_freedom >
		state->config.max_gyro_stddev_rps * state->config.max_gyro_stddev_rps ||
	    state->accel_m2 / degrees_of_freedom >
		state->config.max_accel_stddev_mps2 * state->config.max_accel_stddev_mps2) {
		reset_window(state);
		return NRF54L15_CALIBRATION_MOTION;
	}
	state->complete = true;
	return NRF54L15_CALIBRATION_COMPLETE;
}

int nrf54l15_gyro_calibration_result(const struct nrf54l15_gyro_calibration *state,
	struct nrf54l15_vec3 *bias_rps)
{
	if (state == NULL || bias_rps == NULL || !config_valid(&state->config)) {
		return -EINVAL;
	}
	if (!state->complete) {
		return -EAGAIN;
	}
	*bias_rps = state->gyro_mean;
	return 0;
}
