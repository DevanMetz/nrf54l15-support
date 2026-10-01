/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include <nrf54l15/telemetry.h>

int nrf54l15_control_parse(const void *data, size_t length,
			  enum nrf54l15_control_command *command)
{
	if (data == NULL || command == NULL) {
		return -EINVAL;
	}
	if (length == 4 && memcmp(data, "zero", 4) == 0) {
		*command = NRF54L15_CONTROL_ZERO;
	} else if (length == 5 && memcmp(data, "start", 5) == 0) {
		*command = NRF54L15_CONTROL_START;
	} else if (length == 4 && memcmp(data, "stop", 4) == 0) {
		*command = NRF54L15_CONTROL_STOP;
	} else {
		return -EINVAL;
	}
	return 0;
}

int32_t nrf54l15_scale_i32(float value, float scale)
{
	float scaled = value * scale;

	if (isnan(scaled)) {
		return 0;
	}
	if (scaled >= (float)INT32_MAX) {
		return INT32_MAX;
	}
	if (scaled <= (float)INT32_MIN) {
		return INT32_MIN;
	}
	return (int32_t)roundf(scaled);
}

static int16_t scale_i16(float value, float scale)
{
	int32_t scaled = nrf54l15_scale_i32(value, scale);

	if (scaled > INT16_MAX) {
		return INT16_MAX;
	}
	if (scaled < INT16_MIN) {
		return INT16_MIN;
	}
	return (int16_t)scaled;
}

static void put_u16(uint8_t *out, uint16_t value)
{
	out[0] = (uint8_t)value;
	out[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16(const uint8_t *data)
{
	return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t get_i16(const uint8_t *data)
{
	uint16_t value = get_u16(data);

	/* Avoid implementation-defined unsigned-to-signed conversion. */
	return (int16_t)(value <= INT16_MAX ? (int32_t)value : (int32_t)value - 65536);
}

int nrf54l15_live_encode(uint8_t *out, size_t capacity,
		       uint32_t sequence, uint32_t dt_us,
		       const struct nrf54l15_vec3 *accel,
		       const struct nrf54l15_vec3 *gyro)
{
	if (out == NULL || accel == NULL || gyro == NULL) {
		return -EINVAL;
	}
	if (capacity < NRF54L15_LIVE_FRAME_SIZE) {
		return -EMSGSIZE;
	}
	if (!isfinite(accel->x) || !isfinite(accel->y) || !isfinite(accel->z) ||
	    !isfinite(gyro->x) || !isfinite(gyro->y) || !isfinite(gyro->z)) {
		return -EINVAL;
	}
	out[0] = 'O';
	out[1] = 'F';
	out[2] = 1;
	out[3] = 1;
	put_u16(&out[4], (uint16_t)sequence);
	put_u16(&out[6], (uint16_t)(dt_us > UINT16_MAX ? UINT16_MAX : dt_us));
	put_u16(&out[8], (uint16_t)scale_i16(accel->x, 1000.0f / NRF54L15_GRAVITY_MPS2));
	put_u16(&out[10], (uint16_t)scale_i16(accel->y, 1000.0f / NRF54L15_GRAVITY_MPS2));
	put_u16(&out[12], (uint16_t)scale_i16(accel->z, 1000.0f / NRF54L15_GRAVITY_MPS2));
	put_u16(&out[14], (uint16_t)scale_i16(gyro->x, NRF54L15_RAD_TO_DEG * 16.0f));
	put_u16(&out[16], (uint16_t)scale_i16(gyro->y, NRF54L15_RAD_TO_DEG * 16.0f));
	put_u16(&out[18], (uint16_t)scale_i16(gyro->z, NRF54L15_RAD_TO_DEG * 16.0f));
	return 0;
}

int nrf54l15_live_decode(const uint8_t *data, size_t length,
		       struct nrf54l15_live_frame *out)
{
	if (data == NULL || out == NULL) {
		return -EINVAL;
	}
	if (length != NRF54L15_LIVE_FRAME_SIZE) {
		return -EMSGSIZE;
	}
	if (data[0] != 'O' || data[1] != 'F' || data[2] != 1 || data[3] != 1) {
		return -EBADMSG;
	}
	out->sequence = get_u16(&data[4]);
	out->dt_us = get_u16(&data[6]);
	for (size_t i = 0; i < 3; ++i) {
		out->accel_mg[i] = get_i16(&data[8 + 2 * i]);
		out->gyro_dps_q4[i] = get_i16(&data[14 + 2 * i]);
	}
	return 0;
}
