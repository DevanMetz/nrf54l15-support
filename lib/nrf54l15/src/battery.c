/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <nrf54l15/battery.h>

int nrf54l15_battery_init(struct nrf54l15_battery *battery,
	const struct adc_dt_spec *adc, const struct device *power,
	const struct nrf54l15_battery_config *config)
{
	if (!battery || !adc || !power || !config || !config->output_ohms ||
	    config->full_ohms < config->output_ohms) {
		return -EINVAL;
	}
	if (battery->power_held) {
		return -EBUSY;
	}
	if (battery->ready) {
		return -EALREADY;
	}
	if (!device_is_ready(power)) {
		return -ENODEV;
	}
	if (regulator_is_enabled(power)) {
		return -EBUSY;
	}
	int err = nrf54l15_adc_init(&battery->adc, adc);
	if (err) {
		return err;
	}
	battery->power = power;
	battery->config = *config;
	battery->ready = true;
	return 0;
}

int nrf54l15_battery_release(struct nrf54l15_battery *battery)
{
	if (!battery) {
		return -EINVAL;
	}
	if (!battery->power_held) {
		return 0;
	}
	int err = regulator_disable(battery->power);
	if (!err) {
		battery->power_held = false;
	}
	return err;
}

int nrf54l15_battery_read(struct nrf54l15_battery *battery, bool calibrate,
	struct nrf54l15_battery_sample *sample)
{
	if (!battery || !sample) {
		return -EINVAL;
	}
	if (!battery->ready) {
		return -ENODEV;
	}
	if (battery->power_held || regulator_is_enabled(battery->power)) {
		return -EBUSY;
	}
	int err = regulator_enable(battery->power);
	if (err) {
		return err;
	}
	battery->power_held = true;
	if (battery->config.settle_ms) {
		k_sleep(K_MSEC(battery->config.settle_ms));
	}
	struct nrf54l15_battery_sample next;
	err = nrf54l15_adc_read(&battery->adc, calibrate, &next.input);
	int release_err = nrf54l15_battery_release(battery);
	if (release_err) {
		return release_err;
	}
	if (err) {
		return err;
	}
	if (regulator_is_enabled(battery->power)) {
		return -EBUSY; /* Unexpected external reference; never consume it. */
	}
	int64_t voltage = (int64_t)next.input.millivolts * battery->config.full_ohms;
	int64_t half = battery->config.output_ohms / 2U;
	/* The full int32*uint32 product plus this rounding term fits int64. */
	voltage = (voltage + (voltage < 0 ? -half : half)) / battery->config.output_ohms;
	if (voltage < INT32_MIN || voltage > INT32_MAX) {
		return -ERANGE;
	}
	next.millivolts = (int32_t)voltage;
	*sample = next;
	return 0;
}
