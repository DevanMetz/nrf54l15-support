/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_PACKET_H
#define NRF54L15_SUPPORT_PACKET_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF54L15_PACKET_MAX_PAYLOAD 256U
/* COBS body = payload + 4-byte CRC + at most 2 code bytes. */
#define NRF54L15_PACKET_MAX_ENCODED 262U
#define NRF54L15_PACKET_MAX_WIRE 264U

struct nrf54l15_packet_decoder {
	uint8_t encoded[NRF54L15_PACKET_MAX_ENCODED];
	size_t used;
	bool discard;
};

/* Wire format: 0x00, COBS(payload || little-endian CRC-32/ISO-HDLC), 0x00.
 * Payload length is 0..256, with NULL allowed only for an empty payload.
 * Returns 0 or negative errno. Output and *written are unchanged on error.
 * Payload/output may overlap; written must not overlap either. Capacity need
 * only fit the actual frame; MAX_WIRE always suffices. No heap allocation.
 */
int nrf54l15_packet_encode(const uint8_t *payload, size_t length,
	uint8_t *output, size_t capacity, size_t *written);

/* Required before first feed and whenever bytes were lost upstream. Discards
 * input through the next zero delimiter. Repeated delimiters are harmless;
 * every encoded packet includes a leading delimiter to establish a boundary.
 * No timeout is built in: the caller can reset an abandoned partial packet.
 */
int nrf54l15_packet_reset(struct nrf54l15_packet_decoder *decoder);

/* Feed one byte in stream order. Returns 1 for a complete verified payload
 * (including length 0), 0 while collecting/discarding, -EBADMSG for malformed
 * COBS/CRC, or -EMSGSIZE for an oversized frame or insufficient output space.
 * A frame error consumes/discards that frame; the next boundary permits
 * recovery. An overlong run reports one error then discards through zero.
 * Output and *length change only on return 1. Non-NULL output and length are
 * required on every call; output capacity 0 can accept only empty payloads.
 * -EINVAL does not consume the byte. State, output and length must not overlap.
 * Calls must be serialized. Payload type, sequence, version and delivery
 * acknowledgments belong to the application; CRC is not authentication.
 */
int nrf54l15_packet_feed(struct nrf54l15_packet_decoder *decoder, uint8_t byte,
	uint8_t *output, size_t capacity, size_t *length);

#ifdef __cplusplus
}
#endif
#endif
