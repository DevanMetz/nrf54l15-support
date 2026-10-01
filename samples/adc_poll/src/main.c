/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <nrf54l15/adc.h>

static const struct adc_dt_spec input = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

int main(void)
{
	struct nrf54l15_adc channel;
	int err = nrf54l15_adc_init(&channel, &input);
	if (err) {
		printk("# ADC init failed: %d\n", err);
		return err;
	}
	printk("# ADC,uptime_us,raw,input_millivolts (XIAO D0/A0)\n");
	while (true) {
		struct nrf54l15_adc_sample sample;
		err = nrf54l15_adc_read(&channel, false, &sample);
		if (err) {
			printk("# ADC read failed: %d\n", err);
		} else {
			printk("ADC,%llu,%d,%d\n", (unsigned long long)sample.timestamp_us,
			       sample.raw, sample.millivolts);
		}
		k_sleep(K_MSEC(100));
	}
}
