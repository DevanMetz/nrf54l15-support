/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <nrf54l15/battery.h>

static struct nrf54l15_battery battery;

int main(void)
{
	const struct adc_dt_spec adc = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
	const struct nrf54l15_battery_config config = {
		.full_ohms = 20000,
		.output_ohms = 10000,
		.settle_ms = 100,
	};
	int err = nrf54l15_battery_init(&battery, &adc,
		DEVICE_DT_GET(DT_NODELABEL(vbat_pwr)), &config);
	if (err) {
		printk("# BATTERY init failed: %d\n", err);
		return 0;
	}
	printk("# BATTERY,uptime_us,raw,input_mv,battery_mv\n");
	for (uint32_t count = 0; ; ++count) {
		struct nrf54l15_battery_sample sample;
		/* Calibrate on first read and explicitly again every 60 measurements. */
		err = nrf54l15_battery_read(&battery, count % 60 == 0, &sample);
		if (err) {
			int release_err = nrf54l15_battery_release(&battery);
			printk("# BATTERY read failed: %d; release retry=%d; held=%u\n",
				err, release_err, (unsigned int)battery.power_held);
			return 0;
		}
		printk("BATTERY,%llu,%d,%d,%d\n",
			(unsigned long long)sample.input.timestamp_us, (int)sample.input.raw,
			sample.input.millivolts, sample.millivolts);
		k_sleep(K_MSEC(1000));
	}
}
