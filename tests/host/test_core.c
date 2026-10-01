/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/debounce.h>
#include <nrf54l15/impact.h>
#include <nrf54l15/periodic.h>
#include <nrf54l15/orientation.h>
#include <nrf54l15/telemetry.h>

/* Unlike assert(), these checks run in release builds too. */
#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)
#define NEAR(actual, expected, tolerance) CHECK(fabsf((actual) - (expected)) <= (tolerance))

static void stationary_filter(void)
{
	struct nrf54l15_quat q = {1, 0, 0, 0};
	const struct nrf54l15_vec3 gyro = {0, 0, 0};
	const struct nrf54l15_vec3 accel = {0, 0, NRF54L15_GRAVITY_MPS2};

	/* Regression: zero Madgwick gradient previously produced NaNs. */
	for (int i = 0; i < 20000; ++i) {
		CHECK(nrf54l15_madgwick_update(&q, &gyro, &accel, 0.08f, 1.0f / 416) == 0);
		NEAR(q.w, 1, 1e-6f);
		NEAR(q.x + q.y + q.z, 0, 1e-6f);
	}
	struct nrf54l15_vec3 angles;
	CHECK(nrf54l15_quat_to_euler(&q, &angles) == 0);
	NEAR(angles.x, 0, 1e-4f);
	NEAR(angles.y, 0, 1e-4f);
	NEAR(angles.z, 0, 1e-4f);
}

static void rotating_filter(void)
{
	struct nrf54l15_quat q = {1, 0, 0, 0};
	const struct nrf54l15_vec3 gyro = {0, 0, 90.0f / NRF54L15_RAD_TO_DEG};
	const struct nrf54l15_vec3 accel = {0, 0, 0};
	struct nrf54l15_vec3 angles;

	for (int i = 0; i < 1000; ++i) {
		CHECK(nrf54l15_madgwick_update(&q, &gyro, &accel, 0.08f, 0.001f) == 0);
	}
	CHECK(nrf54l15_quat_to_euler(&q, &angles) == 0);
	NEAR(angles.z, 90, 0.02f);
	NEAR(q.w * q.w + q.z * q.z, 1, 1e-5f);

	/* Gravity correction converges to a known 30 degree roll. */
	q = (struct nrf54l15_quat){1, 0, 0, 0};
	const struct nrf54l15_vec3 still = {0, 0, 0};
	const struct nrf54l15_vec3 tilted = {0, 0.5f, 0.8660254f};
	for (int i = 0; i < 8000; ++i) {
		CHECK(nrf54l15_madgwick_update(&q, &still, &tilted, 0.08f, 1.0f / 416) == 0);
	}
	CHECK(nrf54l15_quat_to_euler(&q, &angles) == 0);
	NEAR(angles.x, 30, 0.06f);
	NEAR(angles.y, 0, 0.01f);
}

static void invalid_filter_inputs(void)
{
	struct nrf54l15_quat q = {1, 0, 0, 0};
	struct nrf54l15_quat original = q;
	struct nrf54l15_vec3 gyro = {0, 0, 0};
	struct nrf54l15_vec3 accel = {0, 0, 1};
	const float bad_dt[] = {0, -1, 0.201f, NAN, INFINITY};

	for (size_t i = 0; i < sizeof(bad_dt) / sizeof(bad_dt[0]); ++i) {
		CHECK(nrf54l15_madgwick_update(&q, &gyro, &accel, 0.08f, bad_dt[i]) == -EINVAL);
		CHECK(memcmp(&q, &original, sizeof(q)) == 0);
	}
	gyro.x = NAN;
	CHECK(nrf54l15_madgwick_update(&q, &gyro, &accel, 0.08f, 0.01f) == -EINVAL);
	CHECK(memcmp(&q, &original, sizeof(q)) == 0);
	gyro.x = 0;
	accel.x = FLT_MAX;
	CHECK(nrf54l15_madgwick_update(&q, &gyro, &accel, 0.08f, 0.01f) == -ERANGE);
	CHECK(memcmp(&q, &original, sizeof(q)) == 0);
	q = (struct nrf54l15_quat){0, 0, 0, 0};
	CHECK(nrf54l15_quat_normalize(&q) == -EINVAL);
	CHECK(nrf54l15_quat_normalize(NULL) == -EINVAL);
	struct nrf54l15_vec3 angles = {7, 8, 9};
	CHECK(nrf54l15_quat_to_euler(&q, &angles) == -EINVAL);
	CHECK(angles.x == 7 && angles.y == 8 && angles.z == 9);
	q = (struct nrf54l15_quat){2, 0, 0, 0};
	CHECK(nrf54l15_quat_normalize(&q) == 0);
	NEAR(q.w, 1, 1e-6f);
	q = (struct nrf54l15_quat){0.7071068f, 0, 0.7071068f, 0};
	CHECK(nrf54l15_quat_to_euler(&q, &angles) == 0);
	NEAR(angles.y, 90, 0.04f);
	NEAR(nrf54l15_angle_delta_deg(-179, 179), 2, 1e-5f);
	NEAR(nrf54l15_angle_delta_deg(179, -179), -2, 1e-5f);
	NEAR(nrf54l15_angle_delta_deg(540, 0), -180, 1e-5f);
	CHECK(isnan(nrf54l15_angle_delta_deg(NAN, 0)));
}

static void telemetry_wire_format(void)
{
	const struct nrf54l15_vec3 accel = {NRF54L15_GRAVITY_MPS2, -NRF54L15_GRAVITY_MPS2, 0};
	const struct nrf54l15_vec3 gyro = {1.0f / NRF54L15_RAD_TO_DEG, -2.0f / NRF54L15_RAD_TO_DEG, 0};
	const uint8_t golden[20] = {
		'O', 'F', 1, 1, 0x34, 0x12, 0x63, 0x09,
		0xe8, 0x03, 0x18, 0xfc, 0, 0, 0x10, 0, 0xe0, 0xff, 0, 0
	};
	uint8_t frame[21];
	memset(frame, 0xa5, sizeof(frame));
	CHECK(nrf54l15_live_encode(frame, 20, 0x1234, 2403, &accel, &gyro) == 0);
	CHECK(memcmp(frame, golden, sizeof(golden)) == 0);
	CHECK(frame[20] == 0xa5);
	struct nrf54l15_live_frame decoded;
	CHECK(nrf54l15_live_decode(frame, 20, &decoded) == 0);
	CHECK(decoded.sequence == 0x1234 && decoded.dt_us == 2403);
	CHECK(decoded.accel_mg[0] == 1000 && decoded.accel_mg[1] == -1000);
	CHECK(decoded.gyro_dps_q4[0] == 16 && decoded.gyro_dps_q4[1] == -32);

	struct nrf54l15_vec3 extreme = {FLT_MAX, -FLT_MAX, 0};
	CHECK(nrf54l15_live_encode(frame, 20, 65536, 100000, &extreme, &extreme) == 0);
	CHECK(nrf54l15_live_decode(frame, 20, &decoded) == 0);
	CHECK(decoded.sequence == 0 && decoded.dt_us == UINT16_MAX);
	CHECK(decoded.accel_mg[0] == INT16_MAX && decoded.accel_mg[1] == INT16_MIN);
	CHECK(decoded.gyro_dps_q4[0] == INT16_MAX && decoded.gyro_dps_q4[1] == INT16_MIN);

	uint8_t saved[21];
	memcpy(saved, frame, sizeof(saved));
	CHECK(nrf54l15_live_encode(frame, 19, 0, 0, &accel, &gyro) == -EMSGSIZE);
	extreme.x = NAN;
	CHECK(nrf54l15_live_encode(frame, 20, 0, 0, &extreme, &gyro) == -EINVAL);
	CHECK(memcmp(frame, saved, sizeof(saved)) == 0);
	struct nrf54l15_live_frame previous = decoded;
	CHECK(nrf54l15_live_decode(frame, 19, &decoded) == -EMSGSIZE);
	CHECK(nrf54l15_live_decode(frame, 21, &decoded) == -EMSGSIZE);
	for (size_t i = 0; i < 4; ++i) {
		frame[i] ^= 0xff;
		CHECK(nrf54l15_live_decode(frame, 20, &decoded) == -EBADMSG);
		frame[i] ^= 0xff;
	}
	CHECK(memcmp(&previous, &decoded, sizeof(decoded)) == 0);
	CHECK(nrf54l15_scale_i32(0.5f, 1) == 1);
	CHECK(nrf54l15_scale_i32(-0.5f, 1) == -1);
	CHECK(nrf54l15_scale_i32(NAN, 1) == 0);
	CHECK(nrf54l15_scale_i32(INFINITY, 1) == INT32_MAX);
	CHECK(nrf54l15_scale_i32(-INFINITY, 1) == INT32_MIN);
}

static void button_bounce(void)
{
	struct nrf54l15_debounce state = {.debounce_ms = 20};
	CHECK(nrf54l15_debounce_update(&state, false, 0) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, true, 1) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, false, 5) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, true, 10) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, true, 29) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, true, 30) == NRF54L15_BUTTON_PRESSED);
	CHECK(nrf54l15_debounce_update(&state, true, 300) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, false, 310) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, true, 315) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, false, 320) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, false, 340) == NRF54L15_BUTTON_RELEASED);

	state = (struct nrf54l15_debounce){.debounce_ms = 20};
	CHECK(nrf54l15_debounce_update(&state, true, UINT32_MAX - 10) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, true, 8) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_debounce_update(&state, true, 9) == NRF54L15_BUTTON_PRESSED);
}

static void control_commands(void)
{
	enum nrf54l15_control_command command = NRF54L15_CONTROL_STOP;
	CHECK(nrf54l15_control_parse("zero", 4, &command) == 0);
	CHECK(command == NRF54L15_CONTROL_ZERO);
	CHECK(nrf54l15_control_parse("start", 5, &command) == 0);
	CHECK(command == NRF54L15_CONTROL_START);
	CHECK(nrf54l15_control_parse("stop", 4, &command) == 0);
	CHECK(command == NRF54L15_CONTROL_STOP);
	CHECK(nrf54l15_control_parse("zero\0garbage", 12, &command) == -EINVAL);
	CHECK(nrf54l15_control_parse("zero\0", 5, &command) == -EINVAL);
	CHECK(nrf54l15_control_parse("ZERO", 4, &command) == -EINVAL);
	CHECK(nrf54l15_control_parse(NULL, 4, &command) == -EINVAL);
	CHECK(nrf54l15_control_parse("", 0, &command) == -EINVAL);
	CHECK(command == NRF54L15_CONTROL_STOP);
}

static void impact_detection(void)
{
	struct nrf54l15_impact state;
	struct nrf54l15_vec3 low = {0, 0, NRF54L15_GRAVITY_MPS2};
	struct nrf54l15_vec3 high = {130, 0, 0};
	CHECK(nrf54l15_impact_init(&state, 117.72f, 40, 800) == 0);
	CHECK(nrf54l15_impact_update(&state, &high, 0) == 1);
	CHECK(nrf54l15_impact_update(&state, &high, 1000) == 0);
	CHECK(nrf54l15_impact_update(&state, &low, 1001) == 0);
	CHECK(nrf54l15_impact_update(&state, &high, 1002) == 1);
	CHECK(nrf54l15_impact_update(&state, &low, 1003) == 0);
	CHECK(nrf54l15_impact_update(&state, &high, 1100) == 0);
	CHECK(nrf54l15_impact_update(&state, &high, 2000) == 0);
	CHECK(nrf54l15_impact_update(&state, &low, 2100) == 0);
	CHECK(nrf54l15_impact_update(&state, &high, 2101) == 1);
	CHECK(nrf54l15_impact_init(&state, 117.72f, 40, 800) == 0);
	CHECK(nrf54l15_impact_update(&state, &high, UINT32_MAX - 400) == 1);
	CHECK(nrf54l15_impact_update(&state, &low, 0) == 0);
	CHECK(nrf54l15_impact_update(&state, &high, 399) == 1);
	high.x = NAN;
	CHECK(nrf54l15_impact_update(&state, &high, 500) == -EINVAL);
	CHECK(nrf54l15_impact_init(&state, 40, 50, 800) == -EINVAL);
	CHECK(nrf54l15_impact_init(&state, INFINITY, 40, 800) == -EINVAL);
}

static void periodic_deadlines(void)
{
	struct nrf54l15_periodic clock;
	CHECK(nrf54l15_periodic_init(&clock, 31250, 416, 500) == 0);
	uint64_t deadline, slots, previous = 500;
	/* Ten exact seconds of fractional tick periods, with no cumulative drift. */
	for (uint64_t i = 1; i <= 4160; ++i) {
		CHECK(nrf54l15_periodic_deadline(&clock, &deadline) == 0);
		CHECK(deadline == 500 + (i * 31250 + 415) / 416);
		CHECK(deadline - previous == 75 || deadline - previous == 76);
		CHECK(nrf54l15_periodic_consume(&clock, deadline - 1, &slots) == 0 && slots == 0);
		CHECK(nrf54l15_periodic_consume(&clock, deadline, &slots) == 0 && slots == 1);
		previous = deadline;
	}
	CHECK(deadline == 313000);
	CHECK(nrf54l15_periodic_consume(&clock, 500 + 11 * 31250, &slots) == 0 && slots == 416);
	CHECK(nrf54l15_periodic_deadline(&clock, &deadline) == 0);
	CHECK(deadline == 500 + 11 * 31250 + 76);
	CHECK(nrf54l15_periodic_consume(&clock, 499, &slots) == -EINVAL);
	CHECK(nrf54l15_periodic_init(&clock, 0, 416, 0) == -EINVAL);
	CHECK(nrf54l15_periodic_init(&clock, 100, 416, 0) == -EINVAL);
	CHECK(nrf54l15_periodic_init(&clock, 31250, 416, UINT64_MAX - 10) == 0);
	CHECK(nrf54l15_periodic_deadline(&clock, &deadline) == -ERANGE);
	CHECK(nrf54l15_periodic_init(&clock, 31250, 416, 0) == 0);
	CHECK(nrf54l15_periodic_consume(&clock, UINT64_C(1000000000000000), &slots) == 0);
	CHECK(nrf54l15_periodic_deadline(&clock, &deadline) == 0);
	CHECK(deadline > UINT64_C(1000000000000000));
}

int main(void)
{
	stationary_filter();
	rotating_filter();
	invalid_filter_inputs();
	telemetry_wire_format();
	button_bounce();
	control_commands();
	impact_detection();
	periodic_deadlines();
	puts("PASS: orientation, invalid inputs, wire format, saturation, debounce, control, impacts, timing");
	return 0;
}
