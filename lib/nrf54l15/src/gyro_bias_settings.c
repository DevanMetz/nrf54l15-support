/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/settings/settings.h>
#include <nrf54l15/gyro_bias.h>

#define GYRO_BIAS_KEY "n54/gyro"

int nrf54l15_gyro_bias_load(uint32_t profile, struct nrf54l15_vec3 *bias_rps)
{
	if (bias_rps == NULL) {
		return -EINVAL;
	}
	int err = settings_subsys_init();
	if (err) {
		return err;
	}
	uint8_t record[NRF54L15_GYRO_BIAS_RECORD_SIZE] = {0};
	ssize_t length = settings_load_one(GYRO_BIAS_KEY, record, sizeof(record));
	if (length < 0) {
		return (int)length;
	}
	if (length == 0) {
		return -ENOENT;
	}
	return nrf54l15_gyro_bias_decode(record, (size_t)length, profile, bias_rps);
}

int nrf54l15_gyro_bias_save(uint32_t profile, const struct nrf54l15_vec3 *bias_rps)
{
	uint8_t record[NRF54L15_GYRO_BIAS_RECORD_SIZE];
	int err = nrf54l15_gyro_bias_encode(record, sizeof(record), profile, bias_rps);
	if (err) {
		return err;
	}
	err = settings_subsys_init();
	if (err) {
		return err;
	}
	return settings_save_one(GYRO_BIAS_KEY, record, sizeof(record));
}

int nrf54l15_gyro_bias_clear(void)
{
	int err = settings_subsys_init();
	return err ? err : settings_delete(GYRO_BIAS_KEY);
}
