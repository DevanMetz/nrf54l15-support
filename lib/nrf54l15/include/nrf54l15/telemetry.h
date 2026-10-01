/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_TELEMETRY_H
#define NRF54L15_SUPPORT_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>
#include <nrf54l15/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF54L15_LIVE_FRAME_SIZE 20

/* OpenFloat v1: OF, version=1, type=1, then little-endian 16-bit fields.
 * Sequence wraps modulo 65536. dt_us and scaled readings saturate.
 * No calibration/shot flags are present in this existing wire format.
 */
struct nrf54l15_live_frame {
	uint16_t sequence;
	uint16_t dt_us;
	int16_t accel_mg[3];
	int16_t gyro_dps_q4[3];
};

enum nrf54l15_control_command {
	NRF54L15_CONTROL_ZERO,
	NRF54L15_CONTROL_START,
	NRF54L15_CONTROL_STOP,
};

/* Exact ASCII command, no terminator, embedded NUL, or whitespace. */
int nrf54l15_control_parse(const void *data, size_t length,
			  enum nrf54l15_control_command *command);

/* Round ties away from zero, saturate overflow, map NaN to zero. */
int32_t nrf54l15_scale_i32(float value, float scale);

/* Input units: m/s^2 and radians/s. Return 0 or negative errno.
 * Invalid input / insufficient capacity leaves output unchanged.
 */
int nrf54l15_live_encode(uint8_t *out, size_t capacity,
		       uint32_t sequence, uint32_t dt_us,
		       const struct nrf54l15_vec3 *accel,
		       const struct nrf54l15_vec3 *gyro);
int nrf54l15_live_decode(const uint8_t *data, size_t length,
		       struct nrf54l15_live_frame *out);

#ifdef __cplusplus
}
#endif
#endif
