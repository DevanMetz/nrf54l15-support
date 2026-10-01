#ifndef TEST_ZEPHYR_ADC_H
#define TEST_ZEPHYR_ADC_H
#include <stdint.h>
#include <zephyr/device.h>
enum adc_reference { ADC_REF_INTERNAL, ADC_REF_EXTERNAL0 };
struct adc_channel_cfg { enum adc_reference reference; };
struct adc_dt_spec {
	const struct device *dev;
	uint8_t channel_id;
	bool channel_cfg_dt_node_exists;
	struct adc_channel_cfg channel_cfg;
	uint8_t resolution, oversampling;
	uint16_t vref_mv;
};
struct adc_sequence {
	void *buffer;
	size_t buffer_size;
	bool calibrate;
	uint32_t channels;
	uint8_t resolution, oversampling;
};
static inline bool adc_is_ready_dt(const struct adc_dt_spec *spec)
{
	return device_is_ready(spec->dev);
}
uint16_t adc_ref_internal(const struct device *);
int adc_channel_setup_dt(const struct adc_dt_spec *);
int adc_sequence_init_dt(const struct adc_dt_spec *, struct adc_sequence *);
int adc_read_dt(const struct adc_dt_spec *, const struct adc_sequence *);
int adc_raw_to_millivolts_dt(const struct adc_dt_spec *, int32_t *);
#endif
