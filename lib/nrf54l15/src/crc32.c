/* SPDX-License-Identifier: Apache-2.0 */
#include <nrf54l15/crc32.h>

uint32_t nrf54l15_crc32(const void *data, size_t length)
{
	const uint8_t *bytes = data;
	uint32_t crc = UINT32_MAX;
	for (size_t i = 0; i < length; ++i) {
		crc ^= bytes[i];
		for (unsigned int bit = 0; bit < 8; ++bit) {
			crc = (crc >> 1) ^ ((crc & 1) ? UINT32_C(0xedb88320) : 0);
		}
	}
	return ~crc;
}
