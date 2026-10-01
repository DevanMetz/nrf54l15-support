/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_ORIENTATION_H
#define NRF54L15_SUPPORT_ORIENTATION_H

#include <nrf54l15/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* All functions return 0 on success, negative errno on invalid inputs.
 * Outputs are unchanged on failure. Caller owns and serializes each state.
 */
int nrf54l15_quat_normalize(struct nrf54l15_quat *q);

/* Gyro: radians/s. Accel: m/s^2 (any consistent scale also works).
 * dt_s: (0, 0.2] seconds. beta: nonnegative correction gain, e.g. 0.08.
 * Zero acceleration means gyro-only integration. Six-axis yaw will drift.
 */
int nrf54l15_madgwick_update(struct nrf54l15_quat *q,
			  const struct nrf54l15_vec3 *gyro,
			  const struct nrf54l15_vec3 *accel,
			  float beta, float dt_s);

/* Euler XYZ roll/pitch/yaw in degrees; pitch is limited to [-90, 90]. */
int nrf54l15_quat_to_euler(const struct nrf54l15_quat *q,
			struct nrf54l15_vec3 *euler_deg);

/* Signed angle minus reference, wrapped to [-180, 180). Nonfinite -> NaN. */
float nrf54l15_angle_delta_deg(float angle, float reference);

#ifdef __cplusplus
}
#endif
#endif
