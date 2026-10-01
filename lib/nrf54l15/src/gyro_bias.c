/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include <nrf54l15/crc32.h>
#include <nrf54l15/gyro_bias.h>

static void put_u32(uint8_t *out, uint32_t value)
{
	for (unsigned int i = 0; i < 4; ++i) {
		out[i] = (uint8_t)(value >> (8 * i));
	}
}

static uint32_t get_u32(const uint8_t *data)
{
	return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
	       ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static float get_bias(const uint8_t *data)
{
	uint32_t raw = get_u32(data);
	int64_t value = raw <= INT32_MAX ? (int64_t)raw : (int64_t)raw - INT64_C(4294967296);
	return (float)((double)value / 1000000.0);
}

int nrf54l15_gyro_bias_encode(uint8_t *out, size_t capacity, uint32_t profile,
			    const struct nrf54l15_vec3 *bias_rps)
{
	if (out == NULL || bias_rps == NULL) {
		return -EINVAL;
	}
	if (capacity < NRF54L15_GYRO_BIAS_RECORD_SIZE) {
		return -EMSGSIZE;
	}
	uint8_t record[NRF54L15_GYRO_BIAS_RECORD_SIZE] = {'G', 'C', 'A', 'L', 1};
	const float values[] = {bias_rps->x, bias_rps->y, bias_rps->z};
	for (size_t i = 0; i < 3; ++i) {
		if (!isfinite(values[i])) {
			return -EINVAL;
		}
		double scaled = round((double)values[i] * 1000000.0);
		if (scaled < INT32_MIN || scaled > INT32_MAX) {
			return -ERANGE;
		}
		put_u32(&record[12 + 4 * i], (uint32_t)(int32_t)scaled);
	}
	put_u32(&record[8], profile);
	put_u32(&record[24], nrf54l15_crc32(record, 24));
	memcpy(out, record, sizeof(record));
	return 0;
}

int nrf54l15_gyro_bias_decode(const uint8_t *data, size_t length, uint32_t profile,
			    struct nrf54l15_vec3 *bias_rps)
{
	if (data == NULL || bias_rps == NULL) {
		return -EINVAL;
	}
	if (length != NRF54L15_GYRO_BIAS_RECORD_SIZE) {
		return -EMSGSIZE;
	}
	if (get_u32(&data[24]) != nrf54l15_crc32(data, 24) || memcmp(data, "GCAL", 4) != 0) {
		return -EBADMSG;
	}
	if (data[4] != 1) {
		return -EPROTONOSUPPORT;
	}
	if (data[5] != 0 || data[6] != 0 || data[7] != 0) {
		return -EBADMSG;
	}
	if (get_u32(&data[8]) != profile) {
		return -EXDEV;
	}
	struct nrf54l15_vec3 next = {get_bias(&data[12]), get_bias(&data[16]), get_bias(&data[20])};
	*bias_rps = next;
	return 0;
}
