/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <nrf54l15/orientation.h>

float nrf54l15_angle_delta_deg(float angle, float reference)
{
	if (!isfinite(angle) || !isfinite(reference)) {
		return NAN;
	}
	float delta = fmodf(fmodf(angle, 360.0f) - fmodf(reference, 360.0f) + 180.0f, 360.0f);
	return (delta < 0 ? delta + 360.0f : delta) - 180.0f;
}

int nrf54l15_quat_normalize(struct nrf54l15_quat *q)
{
	if (q == NULL) {
		return -EINVAL;
	}
	float norm2 = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;

	if (!isfinite(norm2) || norm2 < FLT_MIN) {
		return -EINVAL;
	}
	float scale = 1.0f / sqrtf(norm2);

	q->w *= scale;
	q->x *= scale;
	q->y *= scale;
	q->z *= scale;
	return 0;
}

int nrf54l15_madgwick_update(struct nrf54l15_quat *q,
			  const struct nrf54l15_vec3 *gyro,
			  const struct nrf54l15_vec3 *accel,
			  float beta, float dt_s)
{
	if (q == NULL || gyro == NULL || accel == NULL ||
	    !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.2f ||
	    !isfinite(beta) || beta < 0.0f ||
	    !isfinite(gyro->x) || !isfinite(gyro->y) || !isfinite(gyro->z) ||
	    !isfinite(accel->x) || !isfinite(accel->y) || !isfinite(accel->z)) {
		return -EINVAL;
	}
	struct nrf54l15_quat next = *q;
	if (nrf54l15_quat_normalize(&next) != 0) {
		return -EINVAL;
	}
	float q1 = next.w, q2 = next.x, q3 = next.y, q4 = next.z;
	float gx = gyro->x, gy = gyro->y, gz = gyro->z;
	float ax = accel->x, ay = accel->y, az = accel->z;
	float q_dot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
	float q_dot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
	float q_dot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
	float q_dot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);
	float accel_norm2 = ax * ax + ay * ay + az * az;

	if (!isfinite(accel_norm2)) {
		return -ERANGE;
	}
	if (accel_norm2 >= FLT_MIN && beta > 0.0f) {
		float scale = 1.0f / sqrtf(accel_norm2);
		ax *= scale;
		ay *= scale;
		az *= scale;

		float two_q1 = 2.0f * q1, two_q2 = 2.0f * q2;
		float two_q3 = 2.0f * q3, two_q4 = 2.0f * q4;
		float four_q1 = 4.0f * q1, four_q2 = 4.0f * q2, four_q3 = 4.0f * q3;
		float eight_q2 = 8.0f * q2, eight_q3 = 8.0f * q3;
		float q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3, q4q4 = q4 * q4;
		float s1 = four_q1 * q3q3 + two_q3 * ax + four_q1 * q2q2 - two_q2 * ay;
		float s2 = four_q2 * q4q4 - two_q4 * ax + 4.0f * q1q1 * q2 -
			   two_q1 * ay - four_q2 + eight_q2 * q2q2 +
			   eight_q2 * q3q3 + four_q2 * az;
		float s3 = 4.0f * q1q1 * q3 + two_q1 * ax + four_q3 * q4q4 -
			   two_q4 * ay - four_q3 + eight_q3 * q2q2 +
			   eight_q3 * q3q3 + four_q3 * az;
		float s4 = 4.0f * q2q2 * q4 - two_q2 * ax +
			   4.0f * q3q3 * q4 - two_q3 * ay;
		float gradient_norm2 = s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4;

		/* A perfectly aligned, stationary sensor has no correction.
		 * Normalizing its zero gradient would poison the filter with NaNs.
		 */
		if (gradient_norm2 >= FLT_MIN) {
			scale = 1.0f / sqrtf(gradient_norm2);
			q_dot1 -= beta * (s1 * scale);
			q_dot2 -= beta * (s2 * scale);
			q_dot3 -= beta * (s3 * scale);
			q_dot4 -= beta * (s4 * scale);
		}
	}
	next.w += q_dot1 * dt_s;
	next.x += q_dot2 * dt_s;
	next.y += q_dot3 * dt_s;
	next.z += q_dot4 * dt_s;
	if (nrf54l15_quat_normalize(&next) != 0) {
		return -ERANGE;
	}
	*q = next;
	return 0;
}

int nrf54l15_quat_to_euler(const struct nrf54l15_quat *q,
			struct nrf54l15_vec3 *euler_deg)
{
	if (q == NULL || euler_deg == NULL) {
		return -EINVAL;
	}
	struct nrf54l15_quat unit = *q;
	if (nrf54l15_quat_normalize(&unit) != 0) {
		return -EINVAL;
	}
	float sinr = 2.0f * (unit.w * unit.x + unit.y * unit.z);
	float cosr = 1.0f - 2.0f * (unit.x * unit.x + unit.y * unit.y);
	float sinp = 2.0f * (unit.w * unit.y - unit.z * unit.x);
	float siny = 2.0f * (unit.w * unit.z + unit.x * unit.y);
	float cosy = 1.0f - 2.0f * (unit.y * unit.y + unit.z * unit.z);

	euler_deg->x = atan2f(sinr, cosr) * NRF54L15_RAD_TO_DEG;
	euler_deg->y = fabsf(sinp) >= 1.0f ? copysignf(90.0f, sinp) :
		      asinf(sinp) * NRF54L15_RAD_TO_DEG;
	euler_deg->z = atan2f(siny, cosy) * NRF54L15_RAD_TO_DEG;
	return 0;
}
