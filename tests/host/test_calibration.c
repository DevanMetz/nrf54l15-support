/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/gyro_calibration.h>
#include <nrf54l15/orientation.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)
#define NEAR(a, b, tolerance) CHECK(fabsf((a) - (b)) <= (tolerance))

static const struct nrf54l15_gyro_calibration_config config = {
	.samples_required = 100, .max_gyro_rps = 0.1f,
	.max_accel_error_mps2 = 0.5f, .max_gyro_stddev_rps = 0.005f,
	.max_accel_stddev_mps2 = 0.1f,
};
static const struct nrf54l15_vec3 gravity = {0, 0, NRF54L15_GRAVITY_MPS2};
static const struct nrf54l15_vec3 known_bias = {0.01f, -0.02f, 0.015f};

static void estimate_bias(void)
{
	struct nrf54l15_gyro_calibration state;
	struct nrf54l15_vec3 bias = {7, 8, 9};
	CHECK(nrf54l15_gyro_calibration_begin(&state, &config) == 0);
	CHECK(nrf54l15_gyro_calibration_result(&state, &bias) == -EAGAIN && bias.x == 7);
	for (int i = 0; i < 100; ++i) {
		float noise = i % 2 == 0 ? 0.0001f : -0.0001f;
		struct nrf54l15_vec3 gyro = {known_bias.x + noise, known_bias.y - noise, known_bias.z + noise};
		CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &gyro) == (i == 99 ? 1 : 0));
	}
	CHECK(nrf54l15_gyro_calibration_result(&state, &bias) == 0);
	NEAR(bias.x, known_bias.x, 1e-7f);
	NEAR(bias.y, known_bias.y, 1e-7f);
	NEAR(bias.z, known_bias.z, 1e-7f);
	struct nrf54l15_vec3 moving = {3, 4, 5};
	CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &moving) == 1);
	CHECK(nrf54l15_gyro_calibration_result(&state, &bias) == 0);
	NEAR(bias.x, known_bias.x, 1e-7f);
	CHECK(nrf54l15_gyro_calibration_begin(&state, &state.config) == 0);
	CHECK(state.samples == 0 && !state.complete);
}

static void reject_motion_and_bad_data(void)
{
	struct nrf54l15_gyro_calibration state;
	struct nrf54l15_vec3 gyro = known_bias;
	CHECK(nrf54l15_gyro_calibration_begin(&state, &config) == 0);
	for (int i = 0; i < 99; ++i) {
		CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &gyro) == 0);
	}
	gyro.x = 1;
	CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &gyro) == 2 && state.samples == 0);
	gyro = known_bias;
	CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &gyro) == 0);
	gyro.x = NAN;
	CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &gyro) == -EINVAL && state.samples == 0);
	gyro.x = FLT_MAX;
	CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &gyro) == -ERANGE && state.samples == 0);
	struct nrf54l15_vec3 zero = {0, 0, 0};
	CHECK(nrf54l15_gyro_calibration_update(&state, &zero, &known_bias) == 2);
	struct nrf54l15_gyro_calibration before = state;
	struct nrf54l15_gyro_calibration_config invalid = config;
	invalid.samples_required = 1;
	CHECK(nrf54l15_gyro_calibration_begin(&state, &invalid) == -EINVAL);
	CHECK(memcmp(&state, &before, sizeof(state)) == 0);
	invalid = config;
	invalid.max_gyro_rps = INFINITY;
	CHECK(nrf54l15_gyro_calibration_begin(&state, &invalid) == -EINVAL);
	for (int i = 0; i < 100; ++i) {
		gyro = (struct nrf54l15_vec3){i % 2 == 0 ? 0.02f : -0.02f, 0, 0};
		CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &gyro) == (i == 99 ? 2 : 0));
	}
	CHECK(state.samples == 0 && !state.complete);
	for (int i = 0; i < 100; ++i) {
		struct nrf54l15_vec3 rocking = {i % 2 == 0 ? 0.5f : -0.5f, 0,
			sqrtf(NRF54L15_GRAVITY_MPS2 * NRF54L15_GRAVITY_MPS2 - 0.25f)};
		CHECK(nrf54l15_gyro_calibration_update(&state, &rocking, &known_bias) == (i == 99 ? 2 : 0));
	}
}

static void corrected_yaw_and_maximum_window(void)
{
	struct nrf54l15_gyro_calibration state;
	struct nrf54l15_gyro_calibration_config long_config = config;
	long_config.samples_required = UINT16_MAX;
	struct nrf54l15_vec3 gyro = {0, 0, 0.01f}, bias;
	CHECK(nrf54l15_gyro_calibration_begin(&state, &long_config) == 0);
	for (uint32_t i = 0; i < UINT16_MAX; ++i) {
		CHECK(nrf54l15_gyro_calibration_update(&state, &gravity, &gyro) == (i == UINT16_MAX - 1 ? 1 : 0));
	}
	CHECK(nrf54l15_gyro_calibration_result(&state, &bias) == 0);
	struct nrf54l15_vec3 corrected = {gyro.x - bias.x, gyro.y - bias.y, gyro.z - bias.z};
	struct nrf54l15_quat raw_q = {1, 0, 0, 0}, corrected_q = raw_q;
	for (int i = 0; i < 1000; ++i) {
		CHECK(nrf54l15_madgwick_update(&raw_q, &gyro, &gravity, 0.08f, 0.01f) == 0);
		CHECK(nrf54l15_madgwick_update(&corrected_q, &corrected, &gravity, 0.08f, 0.01f) == 0);
	}
	struct nrf54l15_vec3 raw_angles, corrected_angles;
	CHECK(nrf54l15_quat_to_euler(&raw_q, &raw_angles) == 0);
	CHECK(nrf54l15_quat_to_euler(&corrected_q, &corrected_angles) == 0);
	CHECK(raw_angles.z > 5.7f);
	NEAR(corrected_angles.z, 0, 0.001f);
}

int main(void)
{
	estimate_bias();
	reject_motion_and_bad_data();
	corrected_yaw_and_maximum_window();
	puts("PASS: gyro bias, noise/motion rejection, invalid windows, corrected orientation, 65535 samples");
	return 0;
}
