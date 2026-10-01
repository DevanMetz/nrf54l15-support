#ifndef TEST_ZEPHYR_PWM_H
#define TEST_ZEPHYR_PWM_H
#include <stdint.h>
#include <zephyr/device.h>

typedef uint16_t pwm_flags_t;
#define PWM_POLARITY_NORMAL 0
#define PWM_POLARITY_INVERTED 1
struct pwm_dt_spec {
	const struct device *dev;
	uint32_t channel, period;
	pwm_flags_t flags;
};
static inline bool pwm_is_ready_dt(const struct pwm_dt_spec *spec)
{
	return device_is_ready(spec->dev);
}
int pwm_set_pulse_dt(const struct pwm_dt_spec *spec, uint32_t pulse_ns);
#endif
