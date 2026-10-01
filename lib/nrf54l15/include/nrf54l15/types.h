/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_TYPES_H
#define NRF54L15_SUPPORT_TYPES_H

struct nrf54l15_vec3 {
	float x, y, z;
};

/* Scalar first, body-to-world quaternion. Identity is {1, 0, 0, 0}. */
struct nrf54l15_quat {
	float w, x, y, z;
};

#define NRF54L15_GRAVITY_MPS2 9.80665f
#define NRF54L15_RAD_TO_DEG 57.29577951308232f

#endif
