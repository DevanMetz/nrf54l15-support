/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/crc32.h>
#include <nrf54l15/packet.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static const uint8_t empty_wire[] = {0, 1, 1, 1, 1, 1, 0};
/* Independently generated with Python zlib + zero-separated COBS runs. */
static const uint8_t golden_wire[] = {
	0, 14, '1', '2', '3', '4', '5', '6', '7', '8', '9', 0x26, 0x39, 0xf4, 0xcb, 0
};
static const uint8_t zero_payload[] = {0, 1, 0, 255, 128, 0};
static const uint8_t zero_wire[] = {0, 1, 2, 1, 3, 255, 128, 5, 0xb5, 0x24, 0xb9, 9, 0};
static uint32_t random_state = UINT32_C(0x782bd301);

static uint32_t next_random(void)
{
	random_state ^= random_state << 13;
	random_state ^= random_state >> 17;
	random_state ^= random_state << 5;
	return random_state;
}

static int push(struct nrf54l15_packet_decoder *decoder, uint8_t byte,
	uint8_t *output, size_t capacity, size_t *length)
{
	uint8_t before[NRF54L15_PACKET_MAX_PAYLOAD + 1];
	memcpy(before, output, sizeof(before));
	size_t old_length = *length;
	int result = nrf54l15_packet_feed(decoder, byte, output, capacity, length);
	if (result != 1) {
		CHECK(*length == old_length && !memcmp(output, before, sizeof(before)));
	} else {
		CHECK(*length <= capacity && *length <= NRF54L15_PACKET_MAX_PAYLOAD);
		CHECK(!memcmp(output + *length, before + *length, sizeof(before) - *length));
	}
	CHECK(decoder->used <= NRF54L15_PACKET_MAX_ENCODED);
	return result;
}

static void round_trip(const uint8_t *payload, size_t size)
{
	uint8_t wire[NRF54L15_PACKET_MAX_WIRE + 1];
	uint8_t output[NRF54L15_PACKET_MAX_PAYLOAD + 1];
	memset(wire, 0xa5, sizeof(wire));
	memset(output, 0xcc, sizeof(output));
	size_t wire_size = SIZE_MAX, output_size = SIZE_MAX;
	CHECK(nrf54l15_packet_encode(payload, size, wire, sizeof(wire) - 1, &wire_size) == 0);
	CHECK(wire_size >= 7 && wire_size <= NRF54L15_PACKET_MAX_WIRE);
	CHECK(!wire[0] && !wire[wire_size - 1] && wire[wire_size] == 0xa5);
	struct nrf54l15_packet_decoder decoder;
	CHECK(nrf54l15_packet_reset(&decoder) == 0);
	for (size_t i = 0; i < wire_size; ++i) {
		if (i && i < wire_size - 1) { CHECK(wire[i]); }
		CHECK(push(&decoder, wire[i], output, size, &output_size) == (i == wire_size - 1));
	}
	CHECK(output_size == size);
	if (size) { CHECK(!memcmp(output, payload, size)); }
}

static void test_fixtures(void)
{
	uint8_t wire[NRF54L15_PACKET_MAX_WIRE], all_bytes[256];
	size_t length = 0;
	CHECK(nrf54l15_crc32(NULL, 0) == 0);
	CHECK(nrf54l15_crc32("123456789", 9) == UINT32_C(0xcbf43926));
	CHECK(nrf54l15_packet_encode(NULL, 0, wire, sizeof(wire), &length) == 0);
	CHECK(length == sizeof(empty_wire) && !memcmp(wire, empty_wire, length));
	CHECK(nrf54l15_packet_encode((const uint8_t *)"123456789", 9, wire, sizeof(wire), &length) == 0);
	CHECK(length == sizeof(golden_wire) && !memcmp(wire, golden_wire, length));
	CHECK(nrf54l15_packet_encode(zero_payload, sizeof(zero_payload), wire, sizeof(wire), &length) == 0);
	CHECK(length == sizeof(zero_wire) && !memcmp(wire, zero_wire, length));
	for (size_t i = 0; i < sizeof(all_bytes); ++i) { all_bytes[i] = (uint8_t)i; }
	CHECK(nrf54l15_crc32(all_bytes, sizeof(all_bytes)) == UINT32_C(0x29058c73));
	CHECK(nrf54l15_packet_encode(all_bytes, 256, wire, sizeof(wire), &length) == 0);
	CHECK(length == 264 && wire[1] == 1 && wire[2] == 255 && wire[257] == 6);
	CHECK(!memcmp(wire + 259, "\x73\x8c\x05\x29\x00", 5));
	/* Exactly 254 nonzero raw bytes end with a final empty COBS block. */
	memset(all_bytes, 1, sizeof(all_bytes));
	CHECK(nrf54l15_packet_encode(all_bytes, 250, wire, sizeof(wire), &length) == 0);
	CHECK(length == 258 && wire[1] == 255 && wire[256] == 1 && !wire[257]);
}

static void test_lengths_and_overlap(void)
{
	uint8_t payload[256], alias[NRF54L15_PACKET_MAX_WIRE + 8], expected[NRF54L15_PACKET_MAX_WIRE];
	for (size_t length = 0; length <= sizeof(payload); ++length) {
		memset(payload, 0, sizeof(payload));
		round_trip(payload, length);
		memset(payload, 0xff, sizeof(payload));
		round_trip(payload, length);
		for (size_t i = 0; i < length; ++i) { payload[i] = (uint8_t)i; }
		round_trip(payload, length);
	}
	for (unsigned int byte = 0; byte < 256; ++byte) {
		payload[0] = (uint8_t)byte;
		round_trip(payload, 1);
	}
	for (unsigned int trial = 0; trial < 3000; ++trial) {
		size_t length = next_random() % 257;
		for (size_t i = 0; i < length; ++i) { payload[i] = (uint8_t)next_random(); }
		round_trip(payload, length);
	}
	size_t length, expected_length;
	memset(payload, 0x77, sizeof(payload));
	CHECK(nrf54l15_packet_encode(payload, sizeof(payload), expected, sizeof(expected), &expected_length) == 0);
	for (size_t source_offset = 0; source_offset < 4; ++source_offset) {
		for (size_t target_offset = 0; target_offset < 4; ++target_offset) {
			memcpy(alias + source_offset, payload, sizeof(payload));
			CHECK(nrf54l15_packet_encode(alias + source_offset, sizeof(payload), alias + target_offset,
				expected_length, &length) == 0);
			CHECK(length == expected_length && !memcmp(alias + target_offset, expected, length));
		}
	}
}

static void expect_golden(struct nrf54l15_packet_decoder *decoder)
{
	uint8_t output[257] = {0};
	size_t size = SIZE_MAX;
	for (size_t i = 0; i < sizeof(golden_wire); ++i) {
		int result = push(decoder, golden_wire[i], output, 256, &size);
		CHECK(result != 1 || i == sizeof(golden_wire) - 1);
		if (i == sizeof(golden_wire) - 1) { CHECK(result == 1); }
	}
	CHECK(size == 9 && !memcmp(output, "123456789", 9));
}

static void test_corruption_and_recovery(void)
{
	struct nrf54l15_packet_decoder decoder;
	uint8_t corrupt[sizeof(golden_wire)], output[257] = {0};
	size_t size = SIZE_MAX;
	for (size_t changed = 0; changed < sizeof(corrupt); ++changed) {
		for (unsigned int bit = 0; bit < 8; ++bit) {
			memcpy(corrupt, golden_wire, sizeof(corrupt));
			corrupt[changed] ^= (uint8_t)(1U << bit);
			CHECK(nrf54l15_packet_reset(&decoder) == 0);
			for (size_t i = 0; i < sizeof(corrupt); ++i) {
				CHECK(push(&decoder, corrupt[i], output, 256, &size) != 1);
			}
			expect_golden(&decoder);
		}
	}
	/* Every body-byte deletion is rejected; following packet still works. */
	for (size_t deleted = 1; deleted < sizeof(golden_wire) - 1; ++deleted) {
		CHECK(nrf54l15_packet_reset(&decoder) == 0);
		for (size_t i = 0; i < sizeof(golden_wire); ++i) {
			if (i != deleted) { CHECK(push(&decoder, golden_wire[i], output, 256, &size) != 1); }
		}
		expect_golden(&decoder);
	}
	/* Loss acknowledgment at every offset must abandon the remaining fragment. */
	for (size_t cut = 0; cut < sizeof(golden_wire); ++cut) {
		CHECK(nrf54l15_packet_reset(&decoder) == 0);
		for (size_t i = 0; i < cut; ++i) { CHECK(push(&decoder, golden_wire[i], output, 256, &size) == 0); }
		CHECK(nrf54l15_packet_reset(&decoder) == 0);
		for (size_t i = cut ? cut : 1; i < sizeof(golden_wire); ++i) {
			CHECK(push(&decoder, golden_wire[i], output, 256, &size) == 0);
		}
		expect_golden(&decoder);
	}
	CHECK(nrf54l15_packet_reset(&decoder) == 0);
	for (unsigned int i = 0; i < 100; ++i) { CHECK(push(&decoder, 0, output, 256, &size) == 0); }
	for (unsigned int i = 0; i < 200; ++i) { expect_golden(&decoder); }
}

static void test_bounds_and_errors(void)
{
	struct nrf54l15_packet_decoder decoder;
	uint8_t output[257], wire[NRF54L15_PACKET_MAX_WIRE], before[sizeof(wire)];
	memset(output, 0x5a, sizeof(output));
	memset(wire, 0xa5, sizeof(wire));
	memcpy(before, wire, sizeof(wire));
	size_t size = 6789;
	CHECK(nrf54l15_packet_encode(NULL, 1, wire, sizeof(wire), &size) == -EINVAL);
	CHECK(nrf54l15_packet_encode(output, SIZE_MAX, wire, sizeof(wire), &size) == -EMSGSIZE);
	CHECK(nrf54l15_packet_encode(output, 1, NULL, sizeof(wire), &size) == -EINVAL);
	CHECK(nrf54l15_packet_encode(output, 1, wire, sizeof(wire), NULL) == -EINVAL);
	for (size_t capacity = 0; capacity < sizeof(golden_wire); ++capacity) {
		CHECK(nrf54l15_packet_encode((const uint8_t *)"123456789", 9, wire, capacity, &size) == -EMSGSIZE);
		CHECK(size == 6789 && !memcmp(before, wire, sizeof(wire)));
	}
	CHECK(nrf54l15_packet_reset(NULL) == -EINVAL);
	CHECK(nrf54l15_packet_reset(&decoder) == 0);
	CHECK(nrf54l15_packet_feed(NULL, 0, output, 256, &size) == -EINVAL);
	CHECK(nrf54l15_packet_feed(&decoder, 0, NULL, 256, &size) == -EINVAL);
	CHECK(nrf54l15_packet_feed(&decoder, 0, output, 256, NULL) == -EINVAL);
	CHECK(decoder.discard && !decoder.used);
	CHECK(push(&decoder, 0, output, 256, &size) == 0);
	for (size_t i = 0; i < 10000; ++i) {
		CHECK(push(&decoder, 1, output, 256, &size) == (i == 262 ? -EMSGSIZE : 0));
	}
	expect_golden(&decoder);
	/* 262 one-byte blocks decode to 261 bytes, exceeding raw capacity. */
	for (size_t i = 0; i < 262; ++i) { CHECK(push(&decoder, 1, output, 256, &size) == 0); }
	CHECK(push(&decoder, 0, output, 256, &size) == -EMSGSIZE);
	expect_golden(&decoder);
	CHECK(push(&decoder, 255, output, 256, &size) == 0);
	CHECK(push(&decoder, 0, output, 256, &size) == -EBADMSG);
	expect_golden(&decoder);
	for (size_t i = 0; i < sizeof(golden_wire); ++i) {
		CHECK(push(&decoder, golden_wire[i], output, 8, &size) ==
			(i == sizeof(golden_wire) - 1 ? -EMSGSIZE : 0));
	}
	expect_golden(&decoder);
	for (size_t i = 0; i < sizeof(empty_wire); ++i) {
		CHECK(push(&decoder, empty_wire[i], output, 0, &size) == (i == sizeof(empty_wire) - 1));
	}
	CHECK(!size);
}

static void test_arbitrary_streams(void)
{
	struct guarded_decoder {
		uint64_t before;
		struct nrf54l15_packet_decoder decoder;
		uint64_t after;
	} guarded = {.before = UINT64_MAX, .after = UINT64_MAX};
	uint8_t output[257] = {0};
	size_t size = SIZE_MAX;
	CHECK(nrf54l15_packet_reset(&guarded.decoder) == 0);
	for (unsigned int i = 0; i < 100000; ++i) {
		(void)push(&guarded.decoder, (uint8_t)next_random(), output, 256, &size);
		CHECK(guarded.before == UINT64_MAX && guarded.after == UINT64_MAX);
	}
	expect_golden(&guarded.decoder);
}

static size_t unhex(const char *hex, uint8_t *output, size_t capacity)
{
	size_t length = strlen(hex);
	CHECK(!(length % 2) && length / 2 <= capacity);
	for (size_t i = 0; i < length; ++i) {
		char byte = hex[i];
		unsigned int nibble = byte >= '0' && byte <= '9' ? (unsigned int)(byte - '0') :
			byte >= 'a' && byte <= 'f' ? (unsigned int)(byte - 'a' + 10) : 16;
		CHECK(nibble < 16);
		if (!(i % 2)) { output[i / 2] = (uint8_t)(nibble << 4); }
		else { output[i / 2] |= (uint8_t)nibble; }
	}
	return length / 2;
}

/* Test-only batch interface for Python-generated payload:wire hex fixtures. */
static void interop(void)
{
	char line[2 * NRF54L15_PACKET_MAX_PAYLOAD + 2 * NRF54L15_PACKET_MAX_WIRE + 3];
	unsigned int vectors = 0;
	while (fgets(line, sizeof(line), stdin)) {
		line[strcspn(line, "\r\n")] = 0;
		char *separator = strchr(line, ':');
		CHECK(separator != NULL);
		*separator++ = 0;
		uint8_t payload[256], expected[264], wire[264], output[257];
		memset(output, 0xa5, sizeof(output));
		size_t payload_size = unhex(line, payload, sizeof(payload));
		size_t expected_size = unhex(separator, expected, sizeof(expected));
		size_t wire_size = 0, output_size = SIZE_MAX;
		CHECK(nrf54l15_packet_encode(payload, payload_size, wire, sizeof(wire), &wire_size) == 0);
		CHECK(wire_size == expected_size && !memcmp(wire, expected, wire_size));
		struct nrf54l15_packet_decoder decoder;
		CHECK(nrf54l15_packet_reset(&decoder) == 0);
		for (size_t i = 0; i < expected_size; ++i) {
			CHECK(push(&decoder, expected[i], output, sizeof(payload), &output_size) == (i == expected_size - 1));
		}
		CHECK(output_size == payload_size && !memcmp(output, payload, payload_size));
		++vectors;
	}
	CHECK(!ferror(stdin) && vectors > 0);
	printf("Verified %u Python/C packet vectors\n", vectors);
}

int main(int argc, char **argv)
{
	if (argc == 2 && !strcmp(argv[1], "--interop")) {
		interop();
		return 0;
	}
	CHECK(argc == 1);
	test_fixtures();
	test_lengths_and_overlap();
	test_corruption_and_recovery();
	test_bounds_and_errors();
	test_arbitrary_streams();
	puts("Packet framing and CRC tests passed");
	return 0;
}
