/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/adc.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static struct device dev = {.ready = true};
static int setup_error, init_error, read_error, convert_error, setup_calls, read_calls;
static int16_t raw_result = 2048;
static bool did_calibrate;
static uint16_t reference_mv = 900;

int64_t k_uptime_ticks(void) { return 100200; }
uint16_t adc_ref_internal(const struct device *device)
{
	CHECK(device == &dev);
	return reference_mv;
}
int adc_channel_setup_dt(const struct adc_dt_spec *spec)
{
	CHECK(spec->dev == &dev);
	++setup_calls;
	return setup_error;
}
int adc_sequence_init_dt(const struct adc_dt_spec *spec, struct adc_sequence *sequence)
{
	sequence->channels = 1U << spec->channel_id;
	sequence->resolution = spec->resolution;
	sequence->oversampling = spec->oversampling;
	return init_error;
}
int adc_read_dt(const struct adc_dt_spec *spec, const struct adc_sequence *sequence)
{
	CHECK(sequence->channels == (1U << spec->channel_id));
	CHECK(sequence->resolution == 12 && sequence->oversampling == 8);
	CHECK(sequence->buffer_size == sizeof(int16_t));
	CHECK((uintptr_t)sequence->buffer % 4 == 0);
	*(int16_t *)sequence->buffer = raw_result;
	did_calibrate = sequence->calibrate;
	++read_calls;
	return read_error;
}
int adc_raw_to_millivolts_dt(const struct adc_dt_spec *spec, int32_t *value)
{
	CHECK(spec->channel_cfg.reference == ADC_REF_INTERNAL);
	*value = *value * 3600 / 4096;
	return convert_error;
}

int main(void)
{
	struct adc_dt_spec spec = {
		.dev = &dev, .channel_id = 3, .channel_cfg_dt_node_exists = true,
		.channel_cfg = {.reference = ADC_REF_INTERNAL}, .resolution = 12, .oversampling = 8,
	};
	struct nrf54l15_adc channel;
	struct nrf54l15_adc_sample sample;
	CHECK(nrf54l15_adc_init(&channel, &spec) == 0 && channel.ready);
	CHECK(nrf54l15_adc_read(&channel, false, &sample) == 0 && did_calibrate);
	CHECK(sample.raw == 2048 && sample.millivolts == 1800 && sample.timestamp_us == 100200);
	CHECK(nrf54l15_adc_read(&channel, false, &sample) == 0 && !did_calibrate);
	CHECK(nrf54l15_adc_read(&channel, true, &sample) == 0 && did_calibrate);
	raw_result = -512;
	CHECK(nrf54l15_adc_read(&channel, false, &sample) == 0);
	CHECK(sample.raw == -512 && sample.millivolts == -450);
	unsigned char saved[sizeof(sample)];
	memcpy(saved, &sample, sizeof(sample));
	read_error = -EBUSY;
	CHECK(nrf54l15_adc_read(&channel, true, &sample) == -EBUSY);
	CHECK(!channel.calibrated && memcmp(saved, &sample, sizeof(sample)) == 0);
	read_error = 0;
	CHECK(nrf54l15_adc_read(&channel, false, &sample) == 0 && did_calibrate);
	convert_error = -ENOTSUP;
	CHECK(nrf54l15_adc_read(&channel, false, &sample) == -ENOTSUP);
	CHECK(memcmp(saved, &sample, sizeof(sample)) == 0);
	convert_error = 0;
	init_error = -ENOTSUP;
	int previous_reads = read_calls;
	CHECK(nrf54l15_adc_read(&channel, false, &sample) == -ENOTSUP && previous_reads == read_calls);
	init_error = 0;
	setup_error = -EIO;
	CHECK(nrf54l15_adc_init(&channel, &spec) == -EIO && !channel.ready);
	CHECK(nrf54l15_adc_read(&channel, false, &sample) == -ENODEV);
	setup_error = 0;
	spec.channel_cfg_dt_node_exists = false;
	CHECK(nrf54l15_adc_init(&channel, &spec) == -ENOTSUP);
	spec.channel_cfg_dt_node_exists = true;
	spec.resolution = 16;
	CHECK(nrf54l15_adc_init(&channel, &spec) == -ENOTSUP);
	spec.resolution = 12;
	spec.channel_id = 8;
	CHECK(nrf54l15_adc_init(&channel, &spec) == -EINVAL);
	spec.channel_id = 3;
	reference_mv = 0;
	CHECK(nrf54l15_adc_init(&channel, &spec) == -ENOTSUP);
	reference_mv = 900;
	dev.ready = false;
	CHECK(nrf54l15_adc_init(&channel, &spec) == -ENODEV);
	dev.ready = true;
	CHECK(nrf54l15_adc_init(&channel, &spec) == 0);
	CHECK(nrf54l15_adc_init(&channel, &channel.spec) == 0);
	CHECK(channel.ready && !channel.calibrated && setup_calls == 4);
	puts("PASS: ADC alignment, calibration, voltage conversion, configuration and partial failures");
	return 0;
}
