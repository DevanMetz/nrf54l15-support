/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_TEMPERATURE_H
#define NRF54L15_SUPPORT_TEMPERATURE_H
#include <stdint.h>

struct device;

#ifdef __cplusplus
extern "C" {
#endif

struct nrf54l15_temperature_sample {
	uint64_t timestamp_us; /* Host fetch completion, not the acquisition instant. */
	int32_t millicelsius; /* Signed degrees Celsius * 1000, nearest, ties away from zero. */
};

/* Fetch SENSOR_CHAN_DIE_TEMP through Zephyr. Enable the SoC's temp devicetree
 * node and CONFIG_SENSOR. Output is unchanged on failure; no cached fallback.
 * Thread context only. Serialize all users of this device across fetch/get.
 * The nRF54L15 reports die temperature in 0.25 C steps, not ambient temperature.
 * With CONFIG_TEMP_NRF5_MPSL, takes NCS's shared MPSL lock without waiting:
 * -EBUSY means retry later. Every other TEMP user must obey MPSL ownership too.
 * This API does not impose a measurement timeout. The SDK's non-MPSL backend
 * can wait forever for the clock/conversion IRQ and asserts on clock failures.
 * MPSL also uses a blocking measurement. Do not put this in a hard-deadline path.
 */
int nrf54l15_die_temperature_read(const struct device *device,
	struct nrf54l15_temperature_sample *sample);

#ifdef __cplusplus
}
#endif
#endif
