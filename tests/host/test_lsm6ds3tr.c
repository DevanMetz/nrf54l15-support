/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/lsm6ds3tr.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)
#define NEAR(a, b, tolerance) CHECK(fabsf((a) - (b)) < (tolerance))
static struct device controller = {.ready = true};
static const struct i2c_dt_spec bus = {.bus = &controller, .addr = 0x6a};
static uint8_t regs[256];
static int transactions, fail_transaction;
static bool stuck_reset, corrupt_config;
static uint32_t slept_ms;
static uint16_t fifo[2048], fifo_pattern, pattern_after_read;
static size_t fifo_head, fifo_count;
static uint8_t fifo_flags, flags_after_read;
static bool corrupt_fifo_config, corrupt_fifo_mode;
static bool corrupt_int1, apply_failed_int1_write;
static bool apply_failed_write;
static int corrupt_power_register;

static void clear_fifo(void)
{
	fifo_head = fifo_count = 0;
	fifo_pattern = 0;
	fifo_flags = 0;
}

static void queue_pair(uint16_t first)
{
	CHECK(fifo_head + fifo_count + 6 <= 2048);
	for (unsigned i = 0; i < 6; ++i) {
		fifo[fifo_head + fifo_count++] = (uint16_t)(first + i);
	}
}

int64_t k_uptime_ticks(void) { return 987654; }
void k_sleep(uint32_t ms) { slept_ms += ms; }

static int transaction(const struct i2c_dt_spec *spec)
{
	CHECK(spec->bus == &controller && spec->addr == 0x6a);
	return ++transactions == fail_transaction ? -EIO : 0;
}

int i2c_reg_read_byte_dt(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t *value)
{
	int err = transaction(spec);
	if (err) { return err; }
	if (reg == 0x12 && !stuck_reset) { regs[reg] &= 0xfe; }
	*value = regs[reg];
	if (reg == 0x0a && corrupt_fifo_mode) { *value ^= 1; }
	if (reg == 0x0d && corrupt_int1) { *value ^= 1; }
	if (reg == corrupt_power_register) { *value ^= 1; }
	return 0;
}

int i2c_reg_write_byte_dt(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t value)
{
	int err = transaction(spec);
	if (!err || apply_failed_write || (reg == 0x0d && apply_failed_int1_write)) {
		if (reg == 0x12 && value == 1) {
			memset(regs, 0, sizeof(regs));
			regs[0x0f] = 0x6a;
			clear_fifo();
		}
		if (reg == 0x0a && value == 0) { clear_fifo(); }
		regs[reg] = value;
	}
	return err;
}

int i2c_burst_read_dt(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t *data, uint32_t length)
{
	int err = transaction(spec);
	CHECK((reg == 0x10 && length == 3) || (reg == 0x22 && length == 12) ||
	      (reg == 0x06 && length == 5) || (reg == 0x3a && length == 4) ||
	      (reg == 0x3e && (length == 2 || length == 12)) || (reg == 0x20 && length == 2));
	/* A bus fault can modify the destination before returning an error. */
	if (reg == 0x3a) {
		data[0] = (uint8_t)fifo_count;
		data[1] = (uint8_t)((fifo_count >> 8) | fifo_flags | (fifo_count == 0 ? 0x10 : 0));
		/* Empty FIFO pattern is stale/undefined, including immediately after
		 * the last word of an otherwise aligned pair was popped.
		 */
		data[2] = fifo_count ? (uint8_t)fifo_pattern : 0xff;
		data[3] = fifo_count ? (uint8_t)(fifo_pattern >> 8) : 3;
	} else if (reg == 0x3e) {
		/* The two FIFO output registers expose ONE word. IF_INC continues
		 * into unrelated registers after 0x3f; it does not wrap to 0x3e.
		 * A bus failure may still consume that word before reporting error.
		 */
		CHECK(fifo_count >= 1);
		uint16_t word = fifo[fifo_head++];
		data[0] = (uint8_t)word;
		data[1] = (uint8_t)(word >> 8);
		if (length > 2) { memcpy(data + 2, regs + 0x40, length - 2); }
		--fifo_count;
		fifo_pattern = (uint16_t)((fifo_pattern + 1) % 6);
		fifo_flags |= flags_after_read;
		if (pattern_after_read) { fifo_pattern = pattern_after_read; }
	} else {
		memcpy(data, &regs[reg], length);
	}
	if (reg == 0x20) { regs[0x1e] &= (uint8_t)~0x04; }
	if (reg == 0x10 && corrupt_config) { data[0] ^= 4; }
	if (reg == 0x06 && corrupt_fifo_config) { data[2] ^= 8; }
	if (corrupt_power_register >= reg && corrupt_power_register < reg + (int)length) {
		data[corrupt_power_register - reg] ^= 1;
	}
	return err;
}

static void reset_fake(void)
{
	memset(regs, 0, sizeof(regs));
	regs[0x0f] = 0x6a;
	transactions = fail_transaction = 0;
	stuck_reset = corrupt_config = false;
	slept_ms = 0;
	controller.ready = true;
	clear_fifo();
	pattern_after_read = flags_after_read = 0;
	corrupt_fifo_config = corrupt_fifo_mode = false;
	corrupt_int1 = apply_failed_int1_write = false;
	apply_failed_write = false;
	corrupt_power_register = -1;
}

static void configure_and_fail(void)
{
	const struct nrf54l15_imu_config config = {416, 16, 2000};
	struct nrf54l15_lsm6ds3tr sensor;
	reset_fake();
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == 0);
	CHECK(sensor.ready && regs[0x10] == 0x64 && regs[0x11] == 0x6c && regs[0x12] == 0x44);
	CHECK(slept_ms == 101);
	int successful_transactions = transactions;
	for (int i = 1; i <= successful_transactions; ++i) {
		reset_fake();
		fail_transaction = i;
		CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == -EIO);
		CHECK(!sensor.ready && transactions == i);
	}
	reset_fake();
	stuck_reset = true;
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == -ETIMEDOUT);
	CHECK(!sensor.ready && slept_ms == 100);
	reset_fake();
	regs[0x0f] = 0x69;
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == -ENODEV);
	CHECK(!sensor.ready && transactions == 1);
	reset_fake();
	corrupt_config = true;
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == -EIO);
	CHECK(!sensor.ready);
	reset_fake();
	struct nrf54l15_imu_config bad = {100, 16, 2000};
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &bad) == -EINVAL);
	CHECK(transactions == 0 && !sensor.ready);
	controller.ready = false;
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == -ENODEV);
	CHECK(transactions == 0);
	reset_fake();
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == 0);
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &sensor.bus, &config) == 0);
	CHECK(sensor.ready);
}

static void all_ranges_and_rates(void)
{
	const uint16_t rates[] = {26, 52, 104, 208, 416, 833};
	const uint16_t accel_ranges[] = {2, 4, 8, 16};
	const uint8_t accel_bits[] = {0, 8, 12, 4};
	const uint16_t gyro_ranges[] = {125, 250, 500, 1000, 2000};
	const uint8_t gyro_bits[] = {2, 0, 4, 8, 12};
	for (size_t rate = 0; rate < 6; ++rate) {
		for (size_t accel = 0; accel < 4; ++accel) {
			for (size_t gyro = 0; gyro < 5; ++gyro) {
				reset_fake();
				struct nrf54l15_lsm6ds3tr sensor;
				struct nrf54l15_imu_config config = {rates[rate], accel_ranges[accel], gyro_ranges[gyro]};
				CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == 0);
				CHECK(regs[0x10] == (uint8_t)(((rate + 2) << 4) | accel_bits[accel]));
				CHECK(regs[0x11] == (uint8_t)(((rate + 2) << 4) | gyro_bits[gyro]));
				CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
				CHECK(regs[0x0a] == (uint8_t)(((rate + 2) << 3) | 6));
				CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == 0);
				CHECK(regs[0x10] == accel_bits[accel] && regs[0x11] == gyro_bits[gyro]);
				CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == 0);
				CHECK(regs[0x10] == (uint8_t)(((rate + 2) << 4) | accel_bits[accel]));
				CHECK(regs[0x11] == (uint8_t)(((rate + 2) << 4) | gyro_bits[gyro]));
			}
		}
	}
}

static void fresh_data_and_bus_errors(void)
{
	reset_fake();
	struct nrf54l15_lsm6ds3tr sensor;
	const struct nrf54l15_imu_config config = {416, 16, 2000};
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == 0);
	struct nrf54l15_imu_sample sample;
	memset(&sample, 0x55, sizeof(sample));
	unsigned char before[sizeof(sample)];
	memcpy(before, &sample, sizeof(sample));
	for (uint8_t status = 0; status < 3; ++status) {
		regs[0x1e] = status;
		CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &sample) == -EAGAIN);
		CHECK(memcmp(before, &sample, sizeof(sample)) == 0);
	}
	regs[0x1e] = 3;
	const uint8_t data[12] = {1, 0, 0xff, 0xff, 0, 0x80, 0xff, 0x7f, 0, 0, 0, 0xc0};
	memcpy(&regs[0x22], data, sizeof(data));
	for (int fail = 1; fail <= 2; ++fail) {
		transactions = 0;
		fail_transaction = fail;
		CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &sample) == -EIO);
		CHECK(memcmp(before, &sample, sizeof(sample)) == 0);
	}
	transactions = fail_transaction = 0;
	CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &sample) == 0);
	CHECK(transactions == 2 && sample.timestamp_us == 987654);
	NEAR(sample.gyro_rps.x, 0.0012217305f, 1e-8f);
	NEAR(sample.gyro_rps.y, -0.0012217305f, 1e-8f);
	NEAR(sample.gyro_rps.z, -40.033f, 0.001f);
	NEAR(sample.accel_mps2.x, 156.811f, 0.001f);
	NEAR(sample.accel_mps2.y, 0, 1e-6f);
	NEAR(sample.accel_mps2.z, -78.4079f, 0.001f);
}

static struct nrf54l15_lsm6ds3tr initialized_sensor(void)
{
	struct nrf54l15_lsm6ds3tr sensor;
	const struct nrf54l15_imu_config config = {416, 16, 2000};
	reset_fake();
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == 0);
	transactions = 0;
	return sensor;
}

static void fifo_configuration_and_order(void)
{
	struct nrf54l15_lsm6ds3tr sensor = initialized_sensor();
	struct nrf54l15_lsm6ds3tr_fifo_sample output;
	memset(&output, 0x55, sizeof(output));
	unsigned char before[sizeof(output)];
	memcpy(before, &output, sizeof(output));
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EACCES);
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == 0 && transactions == 0);
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	CHECK(sensor.fifo_active && !sensor.fifo_fault && sensor.discard_next);
	CHECK(regs[6] == 6 && regs[7] == 0 && regs[8] == 9 && regs[9] == 0 && regs[10] == 0x36);
	CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &output.sample) == -EBUSY);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EAGAIN);
	CHECK(memcmp(before, &output, sizeof(output)) == 0);
	queue_pair(10);
	/* Do not consume an incomplete pair, even though FIFO isn't empty. */
	fifo_count = 5;
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EAGAIN);
	CHECK(fifo_count == 5 && sensor.discard_next);
	fifo_count = 6;
	queue_pair(100);
	queue_pair(200);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EAGAIN);
	CHECK(memcmp(before, &output, sizeof(output)) == 0 && fifo_count == 12);
	CHECK(!sensor.discard_next);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == 0);
	CHECK(output.sequence == 0 && output.sample.timestamp_us == 987654);
	NEAR(output.sample.gyro_rps.x, 100 * sensor.gyro_scale, 1e-6f);
	NEAR(output.sample.gyro_rps.y, 101 * sensor.gyro_scale, 1e-6f);
	NEAR(output.sample.gyro_rps.z, 102 * sensor.gyro_scale, 1e-6f);
	NEAR(output.sample.accel_mps2.x, 103 * sensor.accel_scale, 1e-6f);
	NEAR(output.sample.accel_mps2.y, 104 * sensor.accel_scale, 1e-6f);
	NEAR(output.sample.accel_mps2.z, 105 * sensor.accel_scale, 1e-6f);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == 0 && output.sequence == 1);
	NEAR(output.sample.gyro_rps.x, 200 * sensor.gyro_scale, 1e-6f);
	CHECK(fifo_count == 0);
	memcpy(before, &output, sizeof(output));
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EAGAIN);
	CHECK(memcmp(before, &output, sizeof(output)) == 0);
	/* Sequence wraps, independently of signed measurement conversion. */
	sensor.fifo_sequence = UINT32_MAX;
	queue_pair(0x8000);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == 0 && output.sequence == UINT32_MAX);
	NEAR(output.sample.gyro_rps.x, -32768 * sensor.gyro_scale, 1e-5f);
	queue_pair(0xffff);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == 0 && output.sequence == 0);
	NEAR(output.sample.gyro_rps.x, -sensor.gyro_scale, 1e-8f);
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == 0);
	CHECK(!sensor.fifo_active && !sensor.fifo_fault && sensor.discard_next && regs[10] == 0);
	regs[0x1e] = 3;
	CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &output.sample) == -EAGAIN);
	CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &output.sample) == 0);
}

static void fifo_configuration_failures(void)
{
	struct nrf54l15_lsm6ds3tr sensor = initialized_sensor();
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	int operations = transactions;
	for (int fail = 1; fail <= operations; ++fail) {
		sensor = initialized_sensor();
		fail_transaction = fail;
		CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == -EIO);
		CHECK(sensor.fifo_fault && !sensor.fifo_active);
		struct nrf54l15_lsm6ds3tr_fifo_sample output;
		CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EPIPE);
		CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &output.sample) == -EBUSY);
		fail_transaction = 0;
		CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
		CHECK(!sensor.fifo_fault && sensor.fifo_active);
	}
	sensor = initialized_sensor();
	corrupt_fifo_config = true;
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == -EIO && sensor.fifo_fault);
	corrupt_fifo_config = false;
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	for (int fail = 1; fail <= 2; ++fail) {
		transactions = 0;
		fail_transaction = fail;
		CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == -EIO && sensor.fifo_fault);
		fail_transaction = 0;
		CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == 0);
		CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	}
	corrupt_fifo_mode = true;
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == -EIO && sensor.fifo_fault);
	corrupt_fifo_mode = false;
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == 0);
}

static void fifo_large_backlog(void)
{
	struct nrf54l15_lsm6ds3tr sensor = initialized_sensor();
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	for (uint16_t i = 0; i < 43; ++i) {
		queue_pair((uint16_t)(i * 10));
	}
	/* 258 words: the low count byte alone misleadingly says only 2. */
	CHECK(fifo_count == 258);
	struct nrf54l15_lsm6ds3tr_fifo_sample output;
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EAGAIN);
	for (uint16_t i = 1; i < 43; ++i) {
		CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == 0);
		CHECK(output.sequence == (uint32_t)(i - 1));
		NEAR(output.sample.gyro_rps.x, i * 10 * sensor.gyro_scale, 1e-6f);
	}
	CHECK(fifo_count == 0);
	/* New data can arrive without resetting pattern or host sequence. */
	queue_pair(1000);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == 0 && output.sequence == 42);
	NEAR(output.sample.gyro_rps.x, 1000 * sensor.gyro_scale, 1e-6f);
}

static void fifo_read_faults_and_recovery(void)
{
	/* Every read transaction may fail, including after FIFO words were popped. */
	for (int fail = 1; fail <= 8; ++fail) {
		struct nrf54l15_lsm6ds3tr sensor = initialized_sensor();
		CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
		queue_pair(1);
		struct nrf54l15_lsm6ds3tr_fifo_sample output;
		memset(&output, 0x55, sizeof(output));
		unsigned char before[sizeof(output)];
		memcpy(before, &output, sizeof(output));
		transactions = 0;
		fail_transaction = fail;
		CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EIO);
		CHECK(memcmp(before, &output, sizeof(output)) == 0 && sensor.fifo_fault);
		int calls = transactions;
		CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EPIPE && transactions == calls);
		fail_transaction = 0;
		CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
		CHECK(fifo_count == 0 && !sensor.fifo_fault && sensor.fifo_sequence == 0);
		queue_pair(50);
		queue_pair(100);
		CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -EAGAIN);
		CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == 0);
		NEAR(output.sample.gyro_rps.x, 100 * sensor.gyro_scale, 1e-6f);
	}
	/* Report full/overrun and pattern mismatches before OR during the burst. */
	for (int after = 0; after < 2; ++after) {
		for (int fault = 0; fault < 3; ++fault) {
			struct nrf54l15_lsm6ds3tr sensor = initialized_sensor();
			CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
			queue_pair(1);
			if (after) { queue_pair(10); } /* Keep a valid next-word pattern. */
			uint8_t flag = fault == 0 ? 0x40 : fault == 1 ? 0x20 : 0;
			uint16_t pattern = fault == 2 ? 0x101 : 0;
			if (after) {
				flags_after_read = flag;
				pattern_after_read = pattern;
			} else {
				fifo_flags = flag;
				fifo_pattern = pattern;
			}
			struct nrf54l15_lsm6ds3tr_fifo_sample output;
			memset(&output, 0x55, sizeof(output));
			unsigned char before[sizeof(output)];
			memcpy(before, &output, sizeof(output));
			CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) ==
			      (fault == 2 ? -EILSEQ : -EOVERFLOW));
			CHECK(memcmp(before, &output, sizeof(output)) == 0 && sensor.fifo_fault);
		}
	}
	struct nrf54l15_lsm6ds3tr sensor = {0};
	struct nrf54l15_lsm6ds3tr_fifo_sample output;
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(NULL) == -EINVAL);
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(NULL) == -EINVAL);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(NULL, &output) == -EINVAL);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, NULL) == -EINVAL);
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == -ENODEV);
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == -ENODEV);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &output) == -ENODEV);
}

static void fifo_interrupt_routing(void)
{
	struct nrf54l15_lsm6ds3tr sensor = {0};
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(NULL, true) == -EINVAL);
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, false) == -ENODEV);
	sensor = initialized_sensor();
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, true) == -EACCES && !transactions);
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, false) == 0 && !regs[0x0d]);
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	/* No other interrupt source is retained by this dedicated route. */
	regs[0x0d] = 0xff;
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, true) == 0 && regs[0x0d] == 8);
	CHECK(regs[0x12] == 0x44); /* Active-high push-pull from initialization. */
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == 0 && regs[0x0d] == 8);
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0 && regs[0x0d] == 8);
	sensor.fifo_fault = true;
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, true) == -EPIPE);
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, false) == 0 && !regs[0x0d]);
	CHECK(sensor.fifo_fault); /* Routing changes do not repair FIFO continuity. */
	for (int enabled = 0; enabled < 2; ++enabled) {
		for (int failure = 1; failure <= 2; ++failure) {
			sensor = initialized_sensor();
			CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
			regs[0x0d] = enabled ? 0 : 8;
			transactions = 0;
			fail_transaction = failure;
			apply_failed_int1_write = true;
			CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, enabled != 0) == -EIO);
			CHECK(transactions == failure && sensor.ready && sensor.fifo_active && !sensor.fifo_fault);
			CHECK(regs[0x0d] == (enabled ? 8 : 0)); /* Error may follow a wire write. */
			fail_transaction = 0;
			CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, false) == 0 && !regs[0x0d]);
		}
	}
	corrupt_int1 = true;
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, true) == -EIO);
	corrupt_int1 = false;
	const struct nrf54l15_imu_config config = {416, 16, 2000};
	CHECK(nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config) == 0 && !regs[0x0d]);
}

static void expect_acquisition_blocked(struct nrf54l15_lsm6ds3tr *sensor, int error)
{
	struct nrf54l15_imu_sample sample;
	struct nrf54l15_lsm6ds3tr_fifo_sample fifo_sample;
	struct nrf54l15_temperature_sample temperature;
	memset(&sample, 0x53, sizeof(sample));
	memset(&fifo_sample, 0x71, sizeof(fifo_sample));
	memset(&temperature, 0x27, sizeof(temperature));
	unsigned char before[sizeof(sample)], fifo_before[sizeof(fifo_sample)];
	memcpy(before, &sample, sizeof(before));
	memcpy(fifo_before, &fifo_sample, sizeof(fifo_before));
	unsigned char temperature_before[sizeof(temperature)];
	memcpy(temperature_before, &temperature, sizeof(temperature));
	int count = transactions;
	CHECK(nrf54l15_lsm6ds3tr_read(sensor, &sample) == error);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(sensor, &fifo_sample) == error);
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(sensor) == error);
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(sensor, true) == error);
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(sensor, &temperature) == error);
	CHECK(!memcmp(before, &sample, sizeof(before)) && !memcmp(fifo_before, &fifo_sample, sizeof(fifo_before)));
	CHECK(count == transactions);
	CHECK(!memcmp(temperature_before, &temperature, sizeof(temperature)));
}

static void power_cycles_and_stale_data(void)
{
	struct nrf54l15_lsm6ds3tr sensor = {0};
	CHECK(nrf54l15_lsm6ds3tr_suspend(NULL) == -EINVAL);
	CHECK(nrf54l15_lsm6ds3tr_resume(NULL) == -EINVAL);
	CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == -ENODEV);
	CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == -ENODEV);
	sensor = initialized_sensor();
	CHECK(sensor.power == NRF54L15_LSM6DS3TR_RUNNING);
	CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == 0 && !transactions);
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, true) == 0);
	queue_pair(1); queue_pair(11);
	sensor.fifo_sequence = 99;
	uint32_t before_sleep = slept_ms;
	CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == 0);
	CHECK(sensor.ready && sensor.power == NRF54L15_LSM6DS3TR_SUSPENDED);
	CHECK(!sensor.fifo_active && !sensor.fifo_fault && !sensor.fifo_sequence && !fifo_count);
	CHECK(regs[0x0d] == 0 && regs[0x0a] == 0 && regs[0x10] == 4 && regs[0x11] == 12 && regs[0x12] == 0x44);
	CHECK(slept_ms == before_sleep);
	int count = transactions;
	CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == 0 && transactions == count);
	expect_acquisition_blocked(&sensor, -EACCES);
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == 0 && transactions == count);
	CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == 0);
	CHECK(sensor.power == NRF54L15_LSM6DS3TR_RUNNING && sensor.discard_next);
	CHECK(!sensor.fifo_active && !sensor.fifo_fault && regs[0x0d] == 0 && regs[0x0a] == 0);
	CHECK(regs[0x10] == 0x64 && regs[0x11] == 0x6c && regs[0x12] == 0x44);
	CHECK(slept_ms == before_sleep + 100);
	struct nrf54l15_imu_sample sample;
	memset(&sample, 0x71, sizeof(sample));
	unsigned char before[sizeof(sample)];
	memcpy(before, &sample, sizeof(before));
	regs[0x1e] = 3;
	memset(regs + 0x22, 0xa5, 12); /* Old output may survive register power-down. */
	for (int failure = 1; failure <= 2; ++failure) {
		fail_transaction = transactions + failure;
		CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &sample) == -EIO);
		CHECK(sensor.discard_next && !memcmp(before, &sample, sizeof(before)));
	}
	fail_transaction = 0;
	CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &sample) == -EAGAIN && !sensor.discard_next);
	CHECK(!memcmp(before, &sample, sizeof(before)));
	memset(regs + 0x22, 0, 12);
	regs[0x22] = 1;
	CHECK(nrf54l15_lsm6ds3tr_read(&sensor, &sample) == 0);
	NEAR(sample.gyro_rps.x, 0.0012217305f, 1e-8f);
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	queue_pair(10); queue_pair(20);
	struct nrf54l15_lsm6ds3tr_fifo_sample fifo_sample;
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &fifo_sample) == -EAGAIN);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &fifo_sample) == 0 && !fifo_sample.sequence);
	sensor.fifo_fault = true;
	count = transactions;
	CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == 0 && sensor.fifo_fault && transactions == count);
	for (unsigned int cycle = 0; cycle < 100; ++cycle) {
		CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == 0);
		expect_acquisition_blocked(&sensor, -EACCES);
		CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == 0);
		CHECK(sensor.discard_next && !sensor.fifo_fault && !sensor.fifo_active);
	}
}

static struct nrf54l15_lsm6ds3tr prepare_power_transition(bool resume)
{
	struct nrf54l15_lsm6ds3tr sensor = initialized_sensor();
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, true) == 0);
	queue_pair(1);
	if (resume) { CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == 0); }
	transactions = 0;
	return sensor;
}

static void power_transition_failures(void)
{
	for (int resume = 0; resume < 2; ++resume) {
		int (*transition)(struct nrf54l15_lsm6ds3tr *) = resume ?
			nrf54l15_lsm6ds3tr_resume : nrf54l15_lsm6ds3tr_suspend;
		struct nrf54l15_lsm6ds3tr sensor = prepare_power_transition(resume != 0);
		CHECK(transition(&sensor) == 0);
		int stages = transactions;
		for (int partial = 0; partial < 2; ++partial) {
			for (int failure = 1; failure <= stages; ++failure) {
				sensor = prepare_power_transition(resume != 0);
				apply_failed_write = partial != 0;
				fail_transaction = failure;
				uint32_t before_sleep = slept_ms;
				CHECK(transition(&sensor) == -EIO && transactions == failure);
				CHECK(sensor.ready && sensor.power == NRF54L15_LSM6DS3TR_POWER_FAULT);
				CHECK(slept_ms == before_sleep && sensor.fifo_fault && !sensor.fifo_active);
				expect_acquisition_blocked(&sensor, -EPIPE);
				fail_transaction = 0;
				CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == 0);
				CHECK(nrf54l15_lsm6ds3tr_fifo_int1(&sensor, false) == 0);
				expect_acquisition_blocked(&sensor, -EPIPE); /* Cleanup alone is insufficient. */
				CHECK(transition(&sensor) == 0 && !sensor.fifo_fault);
				CHECK(sensor.power == (resume ? NRF54L15_LSM6DS3TR_RUNNING : NRF54L15_LSM6DS3TR_SUSPENDED));
			}
		}
		const int checked_registers[] = {0x10, 0x11, 0x12, 0x0d, 0x0a};
		for (size_t i = 0; i < sizeof(checked_registers) / sizeof(checked_registers[0]); ++i) {
			sensor = prepare_power_transition(resume != 0);
			corrupt_power_register = checked_registers[i];
			CHECK(transition(&sensor) == -EIO);
			expect_acquisition_blocked(&sensor, -EPIPE);
			corrupt_power_register = -1;
			CHECK(transition(&sensor) == 0);
		}
		sensor = prepare_power_transition(resume != 0);
		regs[0x0f] = 0x69;
		CHECK(transition(&sensor) == -ENODEV && transactions == 1);
		expect_acquisition_blocked(&sensor, -EPIPE);
		sensor = prepare_power_transition(resume != 0);
		controller.ready = false;
		CHECK(transition(&sensor) == -ENODEV && transactions == 0);
		controller.ready = true;
		CHECK(transition(&sensor) == 0);
	}
	/* Recovery can also select the opposite state after a partial transition. */
	struct nrf54l15_lsm6ds3tr sensor = prepare_power_transition(false);
	fail_transaction = 6;
	apply_failed_write = true;
	CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == -EIO);
	fail_transaction = 0;
	CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == 0 && regs[0x10] == 0x64 && regs[0x11] == 0x6c);
	CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == 0);
	fail_transaction = transactions + 6;
	CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == -EIO);
	fail_transaction = 0;
	CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == 0 && !(regs[0x10] & 0xf0) && !(regs[0x11] & 0xf0));
}

static void set_temperature(int32_t raw)
{
	uint16_t word = (uint16_t)raw;
	regs[0x20] = (uint8_t)word;
	regs[0x21] = (uint8_t)(word >> 8);
	regs[0x1e] |= 4;
}

static void temperature_conversion_and_freshness(void)
{
	struct nrf54l15_lsm6ds3tr sensor = initialized_sensor();
	struct nrf54l15_temperature_sample sample;
	memset(&sample, 0x29, sizeof(sample));
	unsigned char original[sizeof(sample)];
	memcpy(original, &sample, sizeof(sample));
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(NULL, &sample) == -EINVAL);
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, NULL) == -EINVAL);
	CHECK(!transactions && sensor.temperature_discard_next);
	for (int status = 0; status < 4; ++status) {
		regs[0x1e] = (uint8_t)status;
		CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == -EAGAIN);
		CHECK(!memcmp(original, &sample, sizeof(sample)) && sensor.temperature_discard_next);
	}
	for (int discard = 1; discard >= 0; --discard) {
		for (int failure = 1; failure <= 2; ++failure) {
			transactions = 0;
			fail_transaction = failure;
			set_temperature(256);
			CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == -EIO);
			CHECK(transactions == failure && sensor.temperature_discard_next == (discard != 0));
			CHECK(!memcmp(original, &sample, sizeof(sample)));
		}
		fail_transaction = 0;
		if (discard) {
			set_temperature(0);
			CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == -EAGAIN);
			CHECK(!sensor.temperature_discard_next && !memcmp(original, &sample, sizeof(sample)));
			CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == -EAGAIN);
		}
	}
	int32_t previous = INT32_MIN;
	/* Independent floating-point reference, including every half-millidegree
	 * tie and both signs across all 65,536 two's-complement register values.
	 */
	for (int32_t raw = INT16_MIN; raw <= INT16_MAX; ++raw) {
		set_temperature(raw);
		CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == 0);
		int32_t expected = (int32_t)round(25000.0 + (double)raw * 1000.0 / 256.0);
		CHECK(sample.millicelsius == expected && sample.millicelsius >= previous);
		CHECK(sample.timestamp_us == 987654);
		previous = sample.millicelsius;
	}
	CHECK(!sensor.discard_next && regs[0x1e] == 3); /* Motion status untouched. */
	sensor.ready = false;
	int count = transactions;
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == -ENODEV);
	CHECK(transactions == count);
}

static void temperature_motion_independence(void)
{
	struct nrf54l15_lsm6ds3tr sensor = initialized_sensor();
	struct nrf54l15_temperature_sample sample;
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	queue_pair(10);
	queue_pair(20);
	set_temperature(16);
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == -EAGAIN);
	set_temperature(16);
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == 0);
	CHECK(sample.millicelsius == 25063 && fifo_count == 12 && fifo_head == 0);
	CHECK(sensor.fifo_sequence == 0 && sensor.discard_next && !sensor.fifo_fault);
	/* A real FIFO fault must remain latched even when temperature succeeds. */
	fifo_flags = 0x40;
	struct nrf54l15_lsm6ds3tr_fifo_sample motion;
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &motion) == -EOVERFLOW);
	set_temperature(-16);
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == 0);
	CHECK(sample.millicelsius == 24938 && sensor.fifo_fault && fifo_count == 12);
	CHECK(nrf54l15_lsm6ds3tr_fifo_read(&sensor, &motion) == -EPIPE);
	CHECK(nrf54l15_lsm6ds3tr_suspend(&sensor) == 0);
	set_temperature(-6416); /* Old output may survive register power-down. */
	CHECK(nrf54l15_lsm6ds3tr_resume(&sensor) == 0);
	CHECK(sensor.temperature_discard_next && sensor.discard_next);
	struct nrf54l15_temperature_sample saved = sample;
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == -EAGAIN);
	CHECK(sample.millicelsius == saved.millicelsius && sample.timestamp_us == saved.timestamp_us);
	CHECK(sensor.discard_next && !sensor.temperature_discard_next);
	set_temperature(-6416);
	CHECK(nrf54l15_lsm6ds3tr_temperature_read(&sensor, &sample) == 0 && sample.millicelsius == -63);
	CHECK(sensor.discard_next);
	CHECK(nrf54l15_lsm6ds3tr_fifo_start(&sensor) == 0);
	CHECK(nrf54l15_lsm6ds3tr_fifo_stop(&sensor) == 0);
	CHECK(!sensor.temperature_discard_next); /* FIFO transitions do not reset temperature. */
}

int main(void)
{
	configure_and_fail();
	all_ranges_and_rates();
	fresh_data_and_bus_errors();
	fifo_configuration_and_order();
	fifo_configuration_failures();
	fifo_large_backlog();
	fifo_read_faults_and_recovery();
	fifo_interrupt_routing();
	power_cycles_and_stale_data();
	power_transition_failures();
	temperature_conversion_and_freshness();
	temperature_motion_independence();
	puts("PASS: LSM6DS3TR-C setup, motion, temperature, power transitions and bus fault recovery");
	return 0;
}
