/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/settings/settings.h>
#include <nrf54l15/gyro_bias.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

#define PROFILE UINT32_C(0x12345678)
/* Independent fixture generated using Python struct.pack + zlib.crc32. */
static const uint8_t golden[] = {
	0x47, 0x43, 0x41, 0x4c, 0x01, 0x00, 0x00, 0x00,
	0x78, 0x56, 0x34, 0x12, 0x48, 0xe8, 0x01, 0x00,
	0x70, 0x2f, 0xfc, 0xff, 0x00, 0x00, 0x00, 0x00,
	0xe3, 0xf1, 0x3c, 0xa3,
};
static const struct nrf54l15_vec3 example = {0.125f, -0.25f, 0};
static uint8_t saved[64];
static size_t saved_length;
static int init_error, read_error, write_error, delete_error;
static unsigned int init_calls, read_calls, write_calls, delete_calls;
static bool tear_write;

int settings_subsys_init(void)
{
	++init_calls;
	return init_error;
}

ssize_t settings_load_one(const char *name, void *buf, size_t capacity)
{
	CHECK(strcmp(name, "n54/gyro") == 0);
	++read_calls;
	size_t copied = saved_length < capacity ? saved_length : capacity;
	memcpy(buf, saved, copied);
	/* The real API reports the full record size even if the buffer is smaller. */
	return read_error ? read_error : (ssize_t)saved_length;
}

int settings_save_one(const char *name, const void *value, size_t length)
{
	CHECK(strcmp(name, "n54/gyro") == 0 && length == sizeof(golden));
	++write_calls;
	if (tear_write) {
		memcpy(saved, value, 16);
		saved_length = length;
		return -EIO;
	}
	if (write_error) {
		return write_error;
	}
	memcpy(saved, value, length);
	saved_length = length;
	return 0;
}

int settings_delete(const char *name)
{
	CHECK(strcmp(name, "n54/gyro") == 0);
	++delete_calls;
	if (!delete_error) {
		saved_length = 0;
	}
	return delete_error;
}

static void test_record(void)
{
	uint8_t record[NRF54L15_GYRO_BIAS_RECORD_SIZE + 1];
	memset(record, 0xa5, sizeof(record));
	CHECK(nrf54l15_gyro_bias_encode(record, sizeof(record), PROFILE, &example) == 0);
	CHECK(memcmp(record, golden, sizeof(golden)) == 0 && record[sizeof(golden)] == 0xa5);
	struct nrf54l15_vec3 decoded;
	CHECK(nrf54l15_gyro_bias_decode(golden, sizeof(golden), PROFILE, &decoded) == 0);
	CHECK(decoded.x == example.x && decoded.y == example.y && decoded.z == example.z);
	struct nrf54l15_vec3 sentinel = {7, 8, 9};
	decoded = sentinel;
	CHECK(nrf54l15_gyro_bias_decode(golden, sizeof(golden), PROFILE + 1, &decoded) == -EXDEV);
	CHECK(memcmp(&decoded, &sentinel, sizeof(decoded)) == 0);
	/* Every single-bit corruption, including the CRC itself, must be detected. */
	for (size_t byte = 0; byte < sizeof(golden); ++byte) {
		for (unsigned int bit = 0; bit < 8; ++bit) {
			memcpy(record, golden, sizeof(golden));
			record[byte] ^= (uint8_t)(1U << bit);
			CHECK(nrf54l15_gyro_bias_decode(record, sizeof(golden), PROFILE, &decoded) == -EBADMSG);
			CHECK(memcmp(&decoded, &sentinel, sizeof(decoded)) == 0);
		}
	}
	for (size_t length = 0; length < sizeof(golden); ++length) {
		CHECK(nrf54l15_gyro_bias_decode(golden, length, PROFILE, &decoded) == -EMSGSIZE);
	}
	CHECK(nrf54l15_gyro_bias_decode(record, sizeof(record), PROFILE, &decoded) == -EMSGSIZE);
	/* Valid CRCs for future version and nonzero reserved byte, also generated
	 * independently with zlib.crc32, prevent the checksum from masking checks.
	 */
	memcpy(record, golden, sizeof(golden));
	record[4] = 2;
	const uint8_t future_crc[] = {0x29, 0xbc, 0x95, 0x0c};
	memcpy(record + 24, future_crc, 4);
	CHECK(nrf54l15_gyro_bias_decode(record, sizeof(golden), PROFILE, &decoded) == -EPROTONOSUPPORT);
	memcpy(record, golden, sizeof(golden));
	record[5] = 1;
	const uint8_t reserved_crc[] = {0xbb, 0x71, 0xde, 0x74};
	memcpy(record + 24, reserved_crc, 4);
	CHECK(nrf54l15_gyro_bias_decode(record, sizeof(golden), PROFILE, &decoded) == -EBADMSG);
	CHECK(memcmp(&decoded, &sentinel, sizeof(decoded)) == 0);
	CHECK(nrf54l15_gyro_bias_decode(NULL, sizeof(golden), PROFILE, &decoded) == -EINVAL);
	CHECK(nrf54l15_gyro_bias_decode(golden, sizeof(golden), PROFILE, NULL) == -EINVAL);
}

static void test_quantization_and_invalid_input(void)
{
	uint8_t record[NRF54L15_GYRO_BIAS_RECORD_SIZE];
	/* 1/128 is exactly representable and scales to an exact half-integer. */
	struct nrf54l15_vec3 bias = {1.0f / 128, -1.0f / 128, 0.0000004f};
	struct nrf54l15_vec3 decoded;
	CHECK(nrf54l15_gyro_bias_encode(record, sizeof(record), 0, &bias) == 0);
	CHECK(record[12] == 0x85 && record[13] == 0x1e);
	CHECK(record[16] == 0x7b && record[17] == 0xe1 && record[19] == 0xff);
	CHECK(nrf54l15_gyro_bias_decode(record, sizeof(record), 0, &decoded) == 0);
	CHECK(fabsf(decoded.x - 0.007813f) < 1.0e-9f && decoded.x == -decoded.y && decoded.z == 0);
	bias = (struct nrf54l15_vec3){2147.4836f, -2147.4836f, -0.0f};
	CHECK(nrf54l15_gyro_bias_encode(record, sizeof(record), UINT32_MAX, &bias) == 0);
	CHECK(nrf54l15_gyro_bias_decode(record, sizeof(record), UINT32_MAX, &decoded) == 0);
	CHECK(decoded.x == bias.x && decoded.y == bias.y && decoded.z == 0);
	uint8_t before[sizeof(record)];
	memcpy(before, record, sizeof(record));
	const float invalid[] = {NAN, INFINITY, -INFINITY, FLT_MAX, -FLT_MAX, 2147.484f, -2147.484f};
	for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
		for (size_t axis = 0; axis < 3; ++axis) {
			bias = example;
			if (axis == 0) { bias.x = invalid[i]; }
			if (axis == 1) { bias.y = invalid[i]; }
			if (axis == 2) { bias.z = invalid[i]; }
			CHECK(nrf54l15_gyro_bias_encode(record, sizeof(record), PROFILE, &bias) ==
			      (isfinite(invalid[i]) ? -ERANGE : -EINVAL));
			CHECK(memcmp(record, before, sizeof(record)) == 0);
		}
	}
	CHECK(nrf54l15_gyro_bias_encode(record, sizeof(record) - 1, PROFILE, &example) == -EMSGSIZE);
	CHECK(nrf54l15_gyro_bias_encode(record, sizeof(record), PROFILE, NULL) == -EINVAL);
	CHECK(nrf54l15_gyro_bias_encode(NULL, sizeof(record), PROFILE, &example) == -EINVAL);
	CHECK(memcmp(record, before, sizeof(record)) == 0);
}

static void test_settings(void)
{
	struct nrf54l15_vec3 bias = {7, 8, 9};
	const struct nrf54l15_vec3 sentinel = bias;
	CHECK(nrf54l15_gyro_bias_load(PROFILE, &bias) == -ENOENT);
	CHECK(memcmp(&bias, &sentinel, sizeof(bias)) == 0);
	CHECK(nrf54l15_gyro_bias_save(PROFILE, &example) == 0);
	CHECK(saved_length == sizeof(golden) && memcmp(saved, golden, sizeof(golden)) == 0);
	CHECK(nrf54l15_gyro_bias_load(PROFILE, &bias) == 0);
	CHECK(bias.x == example.x && bias.y == example.y && bias.z == example.z);
	bias = sentinel;
	CHECK(nrf54l15_gyro_bias_load(PROFILE + 1, &bias) == -EXDEV);
	CHECK(memcmp(&bias, &sentinel, sizeof(bias)) == 0);
	read_error = -EIO;
	CHECK(nrf54l15_gyro_bias_load(PROFILE, &bias) == -EIO);
	CHECK(memcmp(&bias, &sentinel, sizeof(bias)) == 0);
	read_error = 0;
	write_error = -ENOSPC;
	CHECK(nrf54l15_gyro_bias_save(PROFILE, &example) == -ENOSPC);
	write_error = 0;
	/* Simulate a backend leaving a partially replaced value after an I/O
	 * failure. This validates rejection, not the physical ZMS recovery logic.
	 */
	tear_write = true;
	CHECK(nrf54l15_gyro_bias_save(PROFILE + 1, &example) == -EIO);
	tear_write = false;
	CHECK(nrf54l15_gyro_bias_load(PROFILE, &bias) == -EBADMSG);
	CHECK(memcmp(&bias, &sentinel, sizeof(bias)) == 0);
	CHECK(nrf54l15_gyro_bias_save(PROFILE, &example) == 0);
	++saved_length;
	CHECK(nrf54l15_gyro_bias_load(PROFILE, &bias) == -EMSGSIZE);
	saved_length = sizeof(golden) - 1;
	CHECK(nrf54l15_gyro_bias_load(PROFILE, &bias) == -EMSGSIZE);
	CHECK(memcmp(&bias, &sentinel, sizeof(bias)) == 0);
	delete_error = -EIO;
	CHECK(nrf54l15_gyro_bias_clear() == -EIO);
	delete_error = 0;
	CHECK(nrf54l15_gyro_bias_clear() == 0 && saved_length == 0);
	CHECK(nrf54l15_gyro_bias_load(PROFILE, &bias) == -ENOENT);
	unsigned int io_before = read_calls + write_calls + delete_calls;
	init_error = -ENODEV;
	CHECK(nrf54l15_gyro_bias_load(PROFILE, &bias) == -ENODEV);
	CHECK(nrf54l15_gyro_bias_save(PROFILE, &example) == -ENODEV);
	CHECK(nrf54l15_gyro_bias_clear() == -ENODEV);
	CHECK(io_before == read_calls + write_calls + delete_calls);
	init_error = 0;
	CHECK(nrf54l15_gyro_bias_save(PROFILE, &example) == 0);
	unsigned int init_before = init_calls;
	io_before = read_calls + write_calls + delete_calls;
	CHECK(nrf54l15_gyro_bias_load(PROFILE, NULL) == -EINVAL);
	CHECK(nrf54l15_gyro_bias_save(PROFILE, NULL) == -EINVAL);
	struct nrf54l15_vec3 invalid = {0, 0, NAN};
	CHECK(nrf54l15_gyro_bias_save(PROFILE, &invalid) == -EINVAL);
	CHECK(init_before == init_calls && io_before == read_calls + write_calls + delete_calls);
}

int main(void)
{
	test_record();
	test_quantization_and_invalid_input();
	test_settings();
	puts("PASS: gyro bias wire fixture, CRC/corruption, version/profile, quantization and Settings failures");
	return 0;
}
