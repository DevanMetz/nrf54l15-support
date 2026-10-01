/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_CRC32_H
#define NRF54L15_SUPPORT_CRC32_H
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC-32/ISO-HDLC: reflected polynomial 0xedb88320, initial/final XOR
 * 0xffffffff. "123456789" gives 0xcbf43926; empty data gives 0.
 * data must address length readable bytes; NULL is allowed only for length 0.
 * Table-free, no allocation. This checksum provides no authentication.
 */
uint32_t nrf54l15_crc32(const void *data, size_t length);

#ifdef __cplusplus
}
#endif
#endif
