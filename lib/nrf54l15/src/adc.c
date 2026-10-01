/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <nrf54l15/adc.h>

int nrf54l15_adc_init(struct nrf54l15_adc *channel, const struct adc_dt_spec *spec)
{
	if (channel == NULL) {
		return -EINVAL;
	}
	channel->ready = false;
	channel->calibrated = false;
	if (spec == NULL || spec->dev == NULL || spec->channel_id >= 8) {
		return -EINVAL;
	}
	if (!spec->channel_cfg_dt_node_exists ||
	    (spec->resolution != 8 && spec->resolution != 10 &&
	     spec->resolution != 12 && spec->resolution != 14)) {
		return -ENOTSUP;
	}
	if (!adc_is_ready_dt(spec)) {
		return -ENODEV;
	}
	uint16_t reference_mv = spec->channel_cfg.reference == ADC_REF_INTERNAL ?
		adc_ref_internal(spec->dev) : spec->vref_mv;
	if (reference_mv == 0) {
		return -ENOTSUP;
	}
	channel->spec = *spec;
	int err = adc_channel_setup_dt(&channel->spec);
	if (!err) {
		channel->ready = true;
	}
	return err;
}

int nrf54l15_adc_read(struct nrf54l15_adc *channel, bool calibrate,
		     struct nrf54l15_adc_sample *sample)
{
	if (channel == NULL || sample == NULL) {
		return -EINVAL;
	}
	if (!channel->ready) {
		return -ENODEV;
	}
	/* Ensure a word-aligned DMA destination without enlarging the sample count. */
	union {
		int32_t alignment;
		int16_t value;
	} raw = {0};
	struct adc_sequence sequence = {
		.buffer = &raw.value,
		.buffer_size = sizeof(raw.value),
		.calibrate = calibrate || !channel->calibrated,
	};
	int err = adc_sequence_init_dt(&channel->spec, &sequence);
	if (err) {
		return err;
	}
	if (sequence.calibrate) {
		channel->calibrated = false;
	}
	err = adc_read_dt(&channel->spec, &sequence);
	if (err) {
		return err;
	}
	channel->calibrated = true;
	struct nrf54l15_adc_sample next = {
		.timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks()),
		.raw = raw.value,
		.millivolts = raw.value,
	};
	err = adc_raw_to_millivolts_dt(&channel->spec, &next.millivolts);
	if (err) {
		return err;
	}
	*sample = next;
	return 0;
}
