/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_OPENFLOAT_BLE_H
#define NRF54L15_SUPPORT_OPENFLOAT_BLE_H

#include <stdbool.h>
#include <stdint.h>
#include <nrf54l15/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Singleton OpenFloat v1 peripheral service (CONFIG_BT_MAX_CONN=1).
 * Service/live/control UUID suffixes: 000000000001/2/3 in
 * 8f3f3b10-0f5a-4f4c-9a2d-xxxxxxxxxxxx. Requires BT_PERIPHERAL=y.
 * Call init from one thread; owns bt_enable and advertising. Initialization
 * can retry after an error without enabling an already enabled stack again.
 * An already active advertiser is success. Do not call init while connected.
 */
int nrf54l15_openfloat_ble_init(void);

/* Thread context only. May block waiting for Bluetooth buffers: run in a
 * transport worker, never in the sample loop, ISR, or Bluetooth callback.
 * Notification subscription and the start/stop control are independent.
 * Returns 0, -EACCES if stopped, -EAGAIN if uninitialized, or stack/codec error.
 * The stack returns -ENOTCONN when there are no subscribed peers.
 */
int nrf54l15_openfloat_ble_send(uint32_t sequence, uint32_t dt_us,
	const struct nrf54l15_vec3 *accel, const struct nrf54l15_vec3 *gyro);

/* Atomically consumes a zero request. Apply calibration in the sample thread.
 * Several requests before the next sample coalesce into one zero operation.
 */
bool nrf54l15_openfloat_ble_take_zero_request(void);

#ifdef __cplusplus
}
#endif
#endif
