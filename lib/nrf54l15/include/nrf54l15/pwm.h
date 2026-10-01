/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_PWM_H
#define NRF54L15_SUPPORT_PWM_H
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/pwm.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_pwm {
	struct pwm_dt_spec spec;
	bool ready;
};

/* Fixed-period nRF PWM output, channel 0..3, normal or inverted polarity.
 * Copy the devicetree configuration and request an inactive output. Success
 * confirms device readiness and that request, not the selected frequency:
 * the SDK checks shared-period/range constraints on a nonconstant waveform.
 * Do not modify the stored spec after initialization.
 */
int nrf54l15_pwm_init(struct nrf54l15_pwm *output, const struct pwm_dt_spec *spec);

/* Active time in nanoseconds, 0..spec.period, with devicetree polarity.
 * Thread context only; serialize all channels on the same controller. Nordic
 * channels share one period, and restarting may wait for a prior period to end.
 * Timing is quantized by the SDK/hardware. Returns 0 or negative errno; an
 * error does not establish the physical pin state. No output readback is kept.
 */
int nrf54l15_pwm_set_pulse_ns(struct nrf54l15_pwm *output, uint32_t pulse_ns);

/* Duty 0..1000 (e.g. 250 = 25%). Rounds the requested active time to the
 * nearest nanosecond, ties up, before the SDK's hardware-cycle quantization.
 * Zero requests inactive; 1000 requests active for the entire period.
 */
int nrf54l15_pwm_set_duty_permille(struct nrf54l15_pwm *output, uint16_t duty);

#ifdef __cplusplus
}
#endif
#endif
