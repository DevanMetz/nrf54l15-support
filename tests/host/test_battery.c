/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/regulator.h>
#include <nrf54l15/battery.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static struct device adc_dev = {.ready = true}, power_dev = {.ready = true};
static struct adc_dt_spec adc_spec = {
	.dev = &adc_dev, .channel_id = 7, .channel_cfg_dt_node_exists = true,
	.channel_cfg = {.reference = ADC_REF_INTERNAL}, .resolution = 12, .oversampling = 8,
};
static const struct nrf54l15_battery_config config = {
	.full_ohms = 20000, .output_ohms = 10000, .settle_ms = 100,
};
static int setup_error, sequence_error, adc_error, convert_error, enable_error, disable_error;
static unsigned int setup_calls, enables, disables, adc_reads, references;
static bool always_on, extra_reference, did_calibrate;
static uint32_t now_ms;
static int16_t raw_value;
static int32_t input_mv;
static char events[128];
static size_t event_count;

static void record(char event)
{
	CHECK(event_count + 1 < sizeof(events));
	events[event_count++] = event;
	events[event_count] = '\0';
}

static void reset_backend(void)
{
	adc_dev.ready = power_dev.ready = true;
	setup_error = sequence_error = adc_error = convert_error = enable_error = disable_error = 0;
	setup_calls = enables = disables = adc_reads = references = now_ms = 0;
	always_on = extra_reference = did_calibrate = false;
	raw_value = 2218;
	input_mv = 1950;
	event_count = 0;
	events[0] = '\0';
}

bool regulator_is_enabled(const struct device *dev)
{
	CHECK(dev == &power_dev);
	return always_on || references != 0;
}

int regulator_enable(const struct device *dev)
{
	CHECK(dev == &power_dev && !always_on);
	record('E');
	++enables;
	++references;
	if (enable_error) { --references; return enable_error; }
	return 0;
}

int regulator_disable(const struct device *dev)
{
	CHECK(dev == &power_dev && references != 0);
	record('D');
	++disables;
	--references;
	/* Match NCS common regulator: failed last release restores its reference. */
	if (disable_error && references == 0) { ++references; return disable_error; }
	return 0;
}

void k_sleep(uint32_t ms)
{
	CHECK(references == 1);
	record('S');
	now_ms += ms;
}

int64_t k_uptime_ticks(void) { return (int64_t)now_ms * 1000 + 17; }
uint16_t adc_ref_internal(const struct device *dev) { CHECK(dev == &adc_dev); return 900; }
int adc_channel_setup_dt(const struct adc_dt_spec *spec)
{
	CHECK(spec->dev == &adc_dev && references == 0);
	++setup_calls;
	return setup_error;
}

int adc_sequence_init_dt(const struct adc_dt_spec *spec, struct adc_sequence *sequence)
{
	CHECK(spec->channel_id == 7 && references == 1);
	record('Q');
	sequence->channels = 1U << spec->channel_id;
	return sequence_error;
}

int adc_read_dt(const struct adc_dt_spec *spec, const struct adc_sequence *sequence)
{
	CHECK(spec->dev == &adc_dev && sequence->channels == (1U << 7));
	CHECK(references == 1 && (uintptr_t)sequence->buffer % 4 == 0);
	record('R');
	++adc_reads;
	did_calibrate = sequence->calibrate;
	*(int16_t *)sequence->buffer = raw_value;
	if (extra_reference) { ++references; }
	return adc_error;
}

int adc_raw_to_millivolts_dt(const struct adc_dt_spec *spec, int32_t *value)
{
	CHECK(spec->dev == &adc_dev && *value == raw_value);
	record('V');
	*value = input_mv;
	return convert_error;
}

static void init_default(struct nrf54l15_battery *battery)
{
	CHECK(nrf54l15_battery_init(battery, &adc_spec, &power_dev, &config) == 0);
	CHECK(battery->ready && !battery->power_held && !battery->adc.calibrated);
}

static void test_initialization(void)
{
	reset_backend();
	struct nrf54l15_battery battery = {0};
	struct nrf54l15_battery_config invalid = config;
	struct nrf54l15_battery_sample sample;
	CHECK(nrf54l15_battery_init(NULL, &adc_spec, &power_dev, &config) == -EINVAL);
	CHECK(nrf54l15_battery_init(&battery, NULL, &power_dev, &config) == -EINVAL);
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, NULL, &config) == -EINVAL);
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, NULL) == -EINVAL);
	invalid.output_ohms = 0;
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &invalid) == -EINVAL);
	invalid.output_ohms = 30000;
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &invalid) == -EINVAL);
	power_dev.ready = false;
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &config) == -ENODEV);
	power_dev.ready = true;
	references = 1; /* Boot-on regulator: preserve that existing ownership. */
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &config) == -EBUSY);
	references = 0;
	always_on = true;
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &config) == -EBUSY);
	always_on = false;
	adc_dev.ready = false;
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &config) == -ENODEV);
	CHECK(setup_calls == 0 && enables == 0 && disables == 0);
	adc_dev.ready = true;
	setup_error = -EIO;
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &config) == -EIO);
	CHECK(!battery.ready && nrf54l15_battery_read(&battery, false, &sample) == -ENODEV);
	CHECK(nrf54l15_battery_read(NULL, false, &sample) == -EINVAL);
	CHECK(nrf54l15_battery_read(&battery, false, NULL) == -EINVAL);
	CHECK(nrf54l15_battery_release(NULL) == -EINVAL);
	CHECK(nrf54l15_battery_release(&battery) == 0 && disables == 0);
	setup_error = 0;
	init_default(&battery);
	CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &config) == -EALREADY);
	CHECK(setup_calls == 2 && enables == 0 && disables == 0);
}

static void test_sample_and_calibration(void)
{
	reset_backend();
	struct nrf54l15_battery battery = {0};
	struct nrf54l15_battery_sample sample;
	init_default(&battery);
	CHECK(nrf54l15_battery_read(&battery, false, &sample) == 0);
	CHECK(strcmp(events, "ESQRVD") == 0 && references == 0 && !battery.power_held);
	CHECK(sample.input.timestamp_us == 100017 && sample.input.raw == 2218);
	CHECK(sample.input.millivolts == 1950 && sample.millivolts == 3900 && did_calibrate);
	CHECK(nrf54l15_battery_read(&battery, false, &sample) == 0 && !did_calibrate);
	CHECK(nrf54l15_battery_read(&battery, true, &sample) == 0 && did_calibrate);
	CHECK(enables == 3 && disables == 3 && references == 0);
	unsigned int saved_disables = disables;
	CHECK(nrf54l15_battery_release(&battery) == 0 && disables == saved_disables);
	/* Snapshot configuration: modifying caller-owned config cannot change reads. */
	CHECK(battery.config.full_ohms == 20000 && battery.config.output_ohms == 10000);
}

static void test_fault_cleanup(void)
{
	for (unsigned int failure = 0; failure < 5; ++failure) {
		reset_backend();
		struct nrf54l15_battery battery = {0};
		struct nrf54l15_battery_sample sample;
		memset(&sample, 0x5a, sizeof(sample));
		unsigned char before[sizeof(sample)];
		memcpy(before, &sample, sizeof(sample));
		init_default(&battery);
		switch (failure) {
		case 0: enable_error = -EIO; break;
		case 1: sequence_error = -EIO; break;
		case 2: adc_error = -EIO; break;
		case 3: convert_error = -EIO; break;
		default: disable_error = -EIO; break;
		}
		CHECK(nrf54l15_battery_read(&battery, true, &sample) == -EIO);
		CHECK(memcmp(before, &sample, sizeof(sample)) == 0);
		CHECK(enables == 1 && disables == (failure == 0 ? 0U : 1U));
		CHECK(references == (failure == 4 ? 1U : 0U));
		if (failure == 4) {
			CHECK(battery.power_held);
			CHECK(nrf54l15_battery_read(&battery, false, &sample) == -EBUSY);
			CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &config) == -EBUSY);
			CHECK(nrf54l15_battery_release(&battery) == -EIO && references == 1);
			CHECK(enables == 1 && adc_reads == 1);
			disable_error = 0;
			CHECK(nrf54l15_battery_release(&battery) == 0 && references == 0 && !battery.power_held);
		}
		enable_error = sequence_error = adc_error = convert_error = disable_error = 0;
		CHECK(nrf54l15_battery_read(&battery, false, &sample) == 0);
		CHECK(references == 0 && !battery.power_held);
		CHECK(did_calibrate == (failure <= 2));
	}
	reset_backend();
	struct nrf54l15_battery battery = {0};
	struct nrf54l15_battery_sample sample;
	init_default(&battery);
	adc_error = -EIO;
	disable_error = -EACCES;
	CHECK(nrf54l15_battery_read(&battery, false, &sample) == -EACCES);
	CHECK(battery.power_held && references == 1 && !battery.adc.calibrated);
	disable_error = 0;
	CHECK(nrf54l15_battery_release(&battery) == 0 && references == 0);
}

static void test_other_owner(void)
{
	reset_backend();
	struct nrf54l15_battery battery = {0};
	struct nrf54l15_battery_sample sample;
	init_default(&battery);
	references = 1;
	CHECK(nrf54l15_battery_read(&battery, false, &sample) == -EBUSY);
	CHECK(enables == 0 && disables == 0 && references == 1);
	CHECK(nrf54l15_battery_release(&battery) == 0 && disables == 0);
	references = 0;
	extra_reference = true;
	memset(&sample, 0x37, sizeof(sample));
	unsigned char before[sizeof(sample)];
	memcpy(before, &sample, sizeof(sample));
	CHECK(nrf54l15_battery_read(&battery, false, &sample) == -EBUSY);
	CHECK(references == 1 && !battery.power_held && memcmp(before, &sample, sizeof(sample)) == 0);
	CHECK(nrf54l15_battery_release(&battery) == 0 && references == 1 && disables == 1);
	CHECK(regulator_disable(&power_dev) == 0); /* The external owner releases its reference. */
	extra_reference = false;
	CHECK(nrf54l15_battery_read(&battery, false, &sample) == 0 && references == 0);
}

static void test_scaling(void)
{
	static const struct {
		uint32_t full, output;
		int32_t input, expected;
		int error;
	} cases[] = {
		{20000, 10000, 2100, 4200, 0}, {20000, 10000, 0, 0, 0},
		{20000, 10000, -1, -2, 0}, {3, 2, 1, 2, 0}, {3, 2, -1, -2, 0},
		{3, 2, 5, 8, 0}, {3, 2, -5, -8, 0}, {4, 3, 1, 1, 0}, {4, 3, -1, -1, 0},
		{UINT32_MAX, UINT32_MAX, INT32_MAX, INT32_MAX, 0},
		{UINT32_MAX, UINT32_MAX, INT32_MIN, INT32_MIN, 0},
		{2, 1, INT32_MAX, 0, -ERANGE}, {2, 1, INT32_MIN, 0, -ERANGE},
		{UINT32_MAX, 1, 1, 0, -ERANGE}, {UINT32_MAX, 1, -1, 0, -ERANGE},
	};
	for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
		reset_backend();
		struct nrf54l15_battery battery = {0};
		struct nrf54l15_battery_sample sample;
		memset(&sample, 0x42, sizeof(sample));
		unsigned char before[sizeof(sample)];
		memcpy(before, &sample, sizeof(sample));
		struct nrf54l15_battery_config cfg = {
			.full_ohms = cases[i].full, .output_ohms = cases[i].output,
			.settle_ms = i % 2 ? 0 : UINT16_MAX,
		};
		input_mv = cases[i].input;
		CHECK(nrf54l15_battery_init(&battery, &adc_spec, &power_dev, &cfg) == 0);
		uint16_t delay = cfg.settle_ms;
		cfg = (struct nrf54l15_battery_config){0};
		CHECK(nrf54l15_battery_read(&battery, false, &sample) == cases[i].error);
		CHECK(now_ms == delay && references == 0 && !battery.power_held);
		if (!cases[i].error) {
			CHECK(sample.millivolts == cases[i].expected);
		} else {
			CHECK(memcmp(before, &sample, sizeof(sample)) == 0);
		}
	}
}

int main(void)
{
	test_initialization();
	test_sample_and_calibration();
	test_fault_cleanup();
	test_other_owner();
	test_scaling();
	puts("battery tests passed");
	return 0;
}
