/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <nrf54l15/pwm.h>

int nrf54l15_pwm_init(struct nrf54l15_pwm *output, const struct pwm_dt_spec *spec)
{
	if (output == NULL) {
		return -EINVAL;
	}
	output->ready = false;
	if (spec == NULL || spec->dev == NULL || spec->period == 0 || spec->channel >= 4) {
		return -EINVAL;
	}
	if ((spec->flags & ~PWM_POLARITY_INVERTED) != 0) {
		return -ENOTSUP;
	}
	if (!pwm_is_ready_dt(spec)) {
		return -ENODEV;
	}
	output->spec = *spec;
	int err = pwm_set_pulse_dt(&output->spec, 0);
	if (!err) {
		output->ready = true;
	}
	return err;
}

int nrf54l15_pwm_set_pulse_ns(struct nrf54l15_pwm *output, uint32_t pulse_ns)
{
	if (output == NULL) {
		return -EINVAL;
	}
	if (!output->ready) {
		return -ENODEV;
	}
	if (pulse_ns > output->spec.period) {
		return -EINVAL;
	}
	return pwm_set_pulse_dt(&output->spec, pulse_ns);
}

int nrf54l15_pwm_set_duty_permille(struct nrf54l15_pwm *output, uint16_t duty)
{
	if (output == NULL || duty > 1000) {
		return -EINVAL;
	}
	if (!output->ready) {
		return -ENODEV;
	}
	uint32_t pulse_ns = (uint32_t)(((uint64_t)output->spec.period * duty + 500) / 1000);
	return nrf54l15_pwm_set_pulse_ns(output, pulse_ns);
}
