/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_SPI_H
#define NRF54L15_SUPPORT_SPI_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF54L15_SPI_MAX_TRANSFER 512

union nrf54l15_spi_buffer {
	uint32_t alignment;
	uint8_t bytes[NRF54L15_SPI_MAX_TRANSFER];
};

struct nrf54l15_spi {
	struct spi_dt_spec spec;
	union nrf54l15_spi_buffer tx;
	union nrf54l15_spi_buffer rx;
	bool ready;
};

/* Zero-initialize before first use. Use persistent storage and do not move,
 * copy, clear, or modify this instance after initialization: the nrfx backend
 * caches configuration pointers between transfers. Each instance describes
 * one fixed bus/device/mode. Serialize calls to an instance in thread context.
 * Initialization validates and copies configuration; hardware configuration
 * is deferred to the first transfer. No probe or wire transaction is performed.
 * Supports 8-bit, full-duplex controller mode, CPOL/CPHA modes 0..3, MSB/LSB,
 * GPIO CS or no CS. Frequency request bounds: 125 kHz..32 MHz. The backend
 * may lower or reject a request that its clock/prescaler cannot represent;
 * on XIAO's SPIM00 use 2 MHz rather than the unsupported 1 MHz request.
 * GPIO CS must be in bus devicetree.
 * Native CS timing, explicit interword delay, half-duplex, multiline, loopback,
 * TI framing, CS hold and bus-lock options return -ENOTSUP. An existing ready
 * instance returns -EALREADY; use a separate persistent instance for another
 * configuration. CS polarity flags must agree with the GPIO specification.
 */
int nrf54l15_spi_init(struct nrf54l15_spi *spi, const struct spi_dt_spec *spec);

/* One CS transaction of 1..NRF54L15_SPI_MAX_TRANSFER bytes. Command/address/
 * dummy/payload bytes are supplied as one complete TX frame by the caller.
 * tx=NULL sends dummy repeatedly; rx=NULL discards input. Both may be NULL
 * for clock-only transfers. Separate aligned RAM buffers support flash or
 * unaligned TX, unaligned RX, and overlapping caller TX/RX (including in-place).
 * Caller buffers must not point into this instance. RX changes only on success.
 * SPI errors propagate without retries; a failed write may have reached the
 * peripheral partially. The SDK supplies completion timeout/abort behavior;
 * waiting for the shared SPI bus lock is not bounded by that timeout.
 */
int nrf54l15_spi_transfer(struct nrf54l15_spi *spi, const void *tx, void *rx,
	size_t length, uint8_t dummy);

#ifdef __cplusplus
}
#endif
#endif
