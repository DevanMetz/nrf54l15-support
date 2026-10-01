/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_BATTERY_H
#define NRF54L15_SUPPORT_BATTERY_H
#include <nrf54l15/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_battery_config {
	uint32_t full_ohms; /* Upper + lower divider resistances. */
	uint32_t output_ohms; /* Lower resistance; >0 and <=full_ohms. */
	uint16_t settle_ms; /* Delay after enabling the divider, before ADC read. */
};

struct nrf54l15_battery {
	struct nrf54l15_adc adc;
	const struct device *power;
	struct nrf54l15_battery_config config;
	bool ready;
	bool power_held; /* A successful regulator_enable still needs releasing. */
};

struct nrf54l15_battery_sample {
	struct nrf54l15_adc_sample input;
	int32_t millivolts; /* Divider-scaled input; not a state-of-charge estimate. */
};

/* Zero-initialize before first use. Supply a switchable regulator dedicated
 * to the measurement divider, initially disabled (no boot-on/always-on).
 * Uses existing Zephyr ADC/regulator backends. Serialize calls and all ADC
 * access in thread context; do not share the regulator with another owner.
 * Rejects an already-enabled regulator instead of releasing another owner's
 * reference. Repeated initialization returns -EALREADY, or -EBUSY if a failed
 * release remains pending. Failed initialization may be retried.
 */
int nrf54l15_battery_init(struct nrf54l15_battery *battery,
	const struct adc_dt_spec *adc, const struct device *power,
	const struct nrf54l15_battery_config *config);

/* Enable, settle, sample (including first-read ADC offset calibration), and
 * release the divider even after an ADC error. Output changes only when the
 * conversion, release and scaling all succeed. Scaling rounds to nearest mV,
 * ties away from zero; overflow returns -ERANGE. Negative input noise remains
 * signed. Explicit calibrate=true requests another ADC offset calibration.
 * A release error takes precedence over an ADC error and keeps power_held set.
 * Further reads return -EBUSY until release succeeds. Enable errors propagate
 * without a disable: the NCS common regulator driver rolls back that reference.
 */
int nrf54l15_battery_read(struct nrf54l15_battery *battery, bool calibrate,
	struct nrf54l15_battery_sample *sample);

/* Retry releasing this object's pending power reference; no-op when none is
 * held. Never release references owned by another client. Do not clear state
 * to bypass a failure. Software reference counts are not physical readback.
 */
int nrf54l15_battery_release(struct nrf54l15_battery *battery);

#ifdef __cplusplus
}
#endif
#endif
