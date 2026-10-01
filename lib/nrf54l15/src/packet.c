/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <nrf54l15/crc32.h>
#include <nrf54l15/packet.h>

int nrf54l15_packet_encode(const uint8_t *payload, size_t length,
	uint8_t *output, size_t capacity, size_t *written)
{
	if ((!payload && length) || !output || !written) { return -EINVAL; }
	if (length > NRF54L15_PACKET_MAX_PAYLOAD) { return -EMSGSIZE; }
	uint8_t wire[NRF54L15_PACKET_MAX_WIRE];
	uint32_t crc = nrf54l15_crc32(payload, length);
	size_t used = 2, code_at = 1;
	uint8_t code = 1;
	wire[0] = 0;
	for (size_t i = 0; i < length + 4; ++i) {
		uint8_t byte = i < length ? payload[i] : (uint8_t)(crc >> (8 * (i - length)));
		if (!byte) {
			wire[code_at] = code;
			code_at = used++;
			code = 1;
		} else {
			wire[used++] = byte;
			if (++code == 0xff) {
				wire[code_at] = code;
				code_at = used++;
				code = 1;
			}
		}
	}
	wire[code_at] = code;
	wire[used++] = 0;
	if (capacity < used) { return -EMSGSIZE; }
	memcpy(output, wire, used);
	*written = used;
	return 0;
}

int nrf54l15_packet_reset(struct nrf54l15_packet_decoder *decoder)
{
	if (!decoder) { return -EINVAL; }
	decoder->used = 0;
	decoder->discard = true;
	return 0;
}

static int decode(const uint8_t *encoded, size_t size, uint8_t *output,
	size_t capacity, size_t *length)
{
	uint8_t raw[NRF54L15_PACKET_MAX_PAYLOAD + 4];
	size_t read_at = 0, used = 0;
	while (read_at < size) {
		uint8_t code = encoded[read_at++];
		size_t count = (size_t)code - 1;
		if (!code || count > size - read_at) { return -EBADMSG; }
		if (count > sizeof(raw) - used) { return -EMSGSIZE; }
		memcpy(raw + used, encoded + read_at, count);
		read_at += count;
		used += count;
		if (code != 0xff && read_at < size) {
			if (used == sizeof(raw)) { return -EMSGSIZE; }
			raw[used++] = 0;
		}
	}
	if (used < 4) { return -EBADMSG; }
	size_t payload_size = used - 4;
	uint32_t expected = (uint32_t)raw[payload_size] |
		((uint32_t)raw[payload_size + 1] << 8) |
		((uint32_t)raw[payload_size + 2] << 16) |
		((uint32_t)raw[payload_size + 3] << 24);
	if (expected != nrf54l15_crc32(raw, payload_size)) { return -EBADMSG; }
	if (capacity < payload_size) { return -EMSGSIZE; }
	memcpy(output, raw, payload_size);
	*length = payload_size;
	return 1;
}

int nrf54l15_packet_feed(struct nrf54l15_packet_decoder *decoder, uint8_t byte,
	uint8_t *output, size_t capacity, size_t *length)
{
	if (!decoder || !output || !length) { return -EINVAL; }
	if (!byte) {
		size_t size = decoder->used;
		decoder->used = 0;
		if (decoder->discard) {
			decoder->discard = false;
			return 0;
		}
		return size ? decode(decoder->encoded, size, output, capacity, length) : 0;
	}
	if (decoder->discard) { return 0; }
	if (decoder->used == sizeof(decoder->encoded)) {
		decoder->used = 0;
		decoder->discard = true;
		return -EMSGSIZE;
	}
	decoder->encoded[decoder->used++] = byte;
	return 0;
}
