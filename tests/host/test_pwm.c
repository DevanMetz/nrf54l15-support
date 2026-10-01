/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <nrf54l15/pwm.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static struct device dev = {.ready = true};
static struct pwm_dt_spec last_spec;
static uint32_t last_pulse;
static unsigned int calls;
static int backend_error;

int pwm_set_pulse_dt(const struct pwm_dt_spec *spec, uint32_t pulse_ns)
{
	CHECK(spec->dev == &dev);
	last_spec = *spec;
	last_pulse = pulse_ns;
	++calls;
	return backend_error;
}

static void test_outputs(void)
{
	struct pwm_dt_spec spec = {.dev = &dev, .period = 1000000};
	struct nrf54l15_pwm output;
	for (uint32_t channel = 0; channel < 4; ++channel) {
		for (pwm_flags_t flags = 0; flags <= PWM_POLARITY_INVERTED; ++flags) {
			spec.channel = channel;
			spec.flags = flags;
			CHECK(nrf54l15_pwm_init(&output, &spec) == 0 && output.ready);
			CHECK(last_pulse == 0 && last_spec.flags == flags);
			CHECK(last_spec.channel == channel && last_spec.period == spec.period);
			CHECK(nrf54l15_pwm_set_duty_permille(&output, 250) == 0);
			CHECK(last_pulse == 250000 && last_spec.flags == flags);
			CHECK(nrf54l15_pwm_set_pulse_ns(&output, 1500) == 0);
			CHECK(last_pulse == 1500 && last_spec.period == spec.period);
			CHECK(nrf54l15_pwm_set_duty_permille(&output, 1000) == 0);
			CHECK(last_pulse == spec.period);
			CHECK(nrf54l15_pwm_set_duty_permille(&output, 0) == 0 && last_pulse == 0);
		}
	}
	/* The instance owns a copy; changing the original cannot retune it. */
	spec.period = 20000000;
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 500) == 0 && last_pulse == 500000);
	CHECK(nrf54l15_pwm_init(&output, &output.spec) == 0 && output.ready);
	CHECK(output.spec.period == 1000000 && last_pulse == 0);
}

static void test_arithmetic(void)
{
	struct pwm_dt_spec spec = {.dev = &dev, .period = 3333};
	struct nrf54l15_pwm output;
	CHECK(nrf54l15_pwm_init(&output, &spec) == 0);
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 333) == 0 && last_pulse == 1110);
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 500) == 0 && last_pulse == 1667);
	/* This mock accepts periods the hardware may reject. Check the full
	 * uint32_t arithmetic range without claiming that range is realizable.
	 */
	const uint32_t periods[] = {1001, 20000, 20000000, UINT32_MAX};
	for (size_t i = 0; i < sizeof(periods) / sizeof(periods[0]); ++i) {
		spec.period = periods[i];
		CHECK(nrf54l15_pwm_init(&output, &spec) == 0);
		uint32_t previous = 0;
		for (uint16_t duty = 0; duty <= 1000; ++duty) {
			CHECK(nrf54l15_pwm_set_duty_permille(&output, duty) == 0);
			CHECK(last_pulse >= previous && last_pulse <= spec.period);
			/* Error relative to ideal active time stays <= half a ns. */
			int64_t error = (int64_t)last_pulse * 1000 - (int64_t)spec.period * duty;
			CHECK(error > -500 && error <= 500);
			previous = last_pulse;
		}
		CHECK(last_pulse == spec.period);
	}
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 999) == 0 && last_pulse == 4290672328U);
}

static void test_errors(void)
{
	struct pwm_dt_spec spec = {.dev = &dev, .period = 1000000};
	struct nrf54l15_pwm output = {0};
	unsigned int before = calls;
	CHECK(nrf54l15_pwm_init(NULL, &spec) == -EINVAL);
	CHECK(nrf54l15_pwm_init(&output, NULL) == -EINVAL && !output.ready);
	CHECK(nrf54l15_pwm_set_pulse_ns(NULL, 0) == -EINVAL);
	CHECK(nrf54l15_pwm_set_duty_permille(NULL, 0) == -EINVAL);
	CHECK(nrf54l15_pwm_set_pulse_ns(&output, 0) == -ENODEV);
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 0) == -ENODEV);
	spec.dev = NULL;
	CHECK(nrf54l15_pwm_init(&output, &spec) == -EINVAL);
	spec.dev = &dev;
	spec.channel = 4;
	CHECK(nrf54l15_pwm_init(&output, &spec) == -EINVAL);
	spec.channel = 0;
	spec.period = 0;
	CHECK(nrf54l15_pwm_init(&output, &spec) == -EINVAL);
	spec.period = 1000000;
	spec.flags = 2;
	CHECK(nrf54l15_pwm_init(&output, &spec) == -ENOTSUP);
	spec.flags = 0;
	dev.ready = false;
	CHECK(nrf54l15_pwm_init(&output, &spec) == -ENODEV);
	dev.ready = true;
	CHECK(calls == before);
	backend_error = -EIO;
	CHECK(nrf54l15_pwm_init(&output, &spec) == -EIO && !output.ready);
	before = calls;
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 500) == -ENODEV && calls == before);
	backend_error = 0;
	CHECK(nrf54l15_pwm_init(&output, &spec) == 0 && output.ready);
	before = calls;
	CHECK(nrf54l15_pwm_set_pulse_ns(&output, spec.period + 1) == -EINVAL);
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 1001) == -EINVAL);
	CHECK(nrf54l15_pwm_set_duty_permille(&output, UINT16_MAX) == -EINVAL);
	CHECK(calls == before);
	/* A driver can accept inactive initialization but reject a nonconstant
	 * waveform, for example when another channel uses a different period.
	 */
	backend_error = -EINVAL;
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 500) == -EINVAL);
	CHECK(output.ready && output.spec.period == spec.period);
	backend_error = -ENOTSUP;
	CHECK(nrf54l15_pwm_set_pulse_ns(&output, 500) == -ENOTSUP);
	backend_error = 0;
	CHECK(nrf54l15_pwm_set_duty_permille(&output, 0) == 0 && last_pulse == 0);
	dev.ready = false;
	CHECK(nrf54l15_pwm_init(&output, &spec) == -ENODEV && !output.ready);
	before = calls;
	CHECK(nrf54l15_pwm_set_pulse_ns(&output, 0) == -ENODEV && calls == before);
	dev.ready = true;
}

int main(void)
{
	test_outputs();
	test_arithmetic();
	test_errors();
	puts("PASS: PWM duty bounds/rounding, fixed period, polarity, initialization and driver errors");
	return 0;
}
