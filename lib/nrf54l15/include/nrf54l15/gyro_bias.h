/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_GYRO_BIAS_H
#define NRF54L15_SUPPORT_GYRO_BIAS_H
#include <stddef.h>
#include <stdint.h>
#include <nrf54l15/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF54L15_GYRO_BIAS_RECORD_SIZE 28

/* Portable v1 record: GCAL, version u8, three reserved zero bytes, profile u32,
 * three signed i32 bias components in microradians/s, then CRC-32/ISO-HDLC of
 * the preceding 24 bytes. All integers are little endian; axes are unchanged.
 * The application assigns a profile ID for its sensor/configuration/axis
 * convention and changes it when an old calibration must no longer be used.
 * Finite values that cannot be represented are rejected, never saturated.
 * Encoding rounds to nearest microradian/s, ties away from zero.
 * Output stays unchanged on any error. CRC detects corruption, not authenticity.
 */
int nrf54l15_gyro_bias_encode(uint8_t *out, size_t capacity, uint32_t profile,
			    const struct nrf54l15_vec3 *bias_rps);
int nrf54l15_gyro_bias_decode(const uint8_t *data, size_t length, uint32_t profile,
			    struct nrf54l15_vec3 *bias_rps);

/* Zephyr Settings adapter, available with CONFIG_SETTINGS. One record under
 * the reserved key "n54/gyro"; no settings handlers or global bias state.
 * Each call ensures the Settings subsystem is initialized. Use thread context
 * outside acquisition loops; storage can block. Serialize this record's use.
 * Recommended nRF54L15 backend: SETTINGS_ZMS with ZMS_DATA_CRC enabled.
 * load: -ENOENT not found by the backend, -EMSGSIZE wrong length, -EBADMSG corruption,
 * -EPROTONOSUPPORT unknown version, -EXDEV different profile, or backend error.
 * load never changes output on failure and never repairs/overwrites a record.
 * The NCS 3.3.0 ZMS name lookup can hide read failures as missing keys; callers
 * must not interpret -ENOENT alone as a reason to replace or format storage.
 * save/clear affect only this key. Backend failures do not prove whether the
 * previous record survived; reload before using a value after a failed write.
 */
int nrf54l15_gyro_bias_load(uint32_t profile, struct nrf54l15_vec3 *bias_rps);
int nrf54l15_gyro_bias_save(uint32_t profile, const struct nrf54l15_vec3 *bias_rps);
int nrf54l15_gyro_bias_clear(void);

#ifdef __cplusplus
}
#endif
#endif
