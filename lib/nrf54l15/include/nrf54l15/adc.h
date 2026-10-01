/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_ADC_H
#define NRF54L15_SUPPORT_ADC_H
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_adc {
	struct adc_dt_spec spec;
	bool ready;
	bool calibrated;
};

struct nrf54l15_adc_sample {
	uint64_t timestamp_us; /* Host time at conversion completion. */
	int16_t raw;
	int32_t millivolts; /* ADC input voltage, before any external divider scaling. */
};

/* Single-channel, synchronous nRF SAADC adapter. Configuration comes entirely
 * from devicetree, including gain/reference/input/oversampling. Supports the
 * SDK's 8/10/12/14-bit resolutions. Serialize use of the ADC peripheral.
 */
int nrf54l15_adc_init(struct nrf54l15_adc *channel, const struct adc_dt_spec *spec);

/* Thread context: blocks until one conversion completes. The first successful
 * read calibrates; calibrate=true requests another offset calibration.
 * Returns 0 or negative errno. Failure leaves the sample unchanged.
 */
int nrf54l15_adc_read(struct nrf54l15_adc *channel, bool calibrate,
		     struct nrf54l15_adc_sample *sample);

#ifdef __cplusplus
}
#endif
#endif
