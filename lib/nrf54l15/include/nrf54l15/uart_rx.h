/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_UART_RX_H
#define NRF54L15_SUPPORT_UART_RX_H
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/sys/ring_buffer.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF54L15_UART_RX_MAX_CAPACITY 32767U
#define NRF54L15_UART_RX_MAX_READ 256U

struct nrf54l15_uart_rx_stats {
	uint32_t received_bytes; /* Bytes actually returned by uart_fifo_read. */
	uint32_t discarded_bytes; /* Software drops, including discarded backlog. */
	uint32_t overflow_events;
	uint32_t uart_error_events;
	uint32_t uart_error_mask; /* OR of Zephyr UART_ERROR_* / UART_BREAK bits. */
	int fault; /* First negative IRQ/FIFO API failure; requires stop/start. */
	bool gap_pending;
	bool running; /* Reception requested, not physical hardware readback. */
};

struct nrf54l15_uart_rx {
	const struct device *dev;
	struct ring_buf ring;
	struct nrf54l15_uart_rx_stats stats;
	bool bound;
};

/* NCS nrfx interrupt-driven UART on the single-core nRF54L15 application CPU.
 * Requires CONFIG_UART_INTERRUPT_DRIVEN, CONFIG_RING_BUFFER and the selected
 * UART_<instance>_INTERRUPT_DRIVEN option. Baud/framing come from devicetree.
 * XIAO UART21 on P2.08/P2.07 crosses power domains: keep constant latency
 * enabled (CONFIG_NRF_SYS_EVENT=y, CONFIG_SOC_NRF_FORCE_CONSTLAT=y in the
 * examples). This costs power; the adapter does not manage that SoC policy.
 * Zero-initialize state. Own the UART IRQ callback and RX/error interrupts
 * exclusively; never use the console, shell, async UART or IRQ-driven TX on
 * that device. Polling TX via uart_poll_out is allowed. Serialize public calls
 * in thread context. State/storage must stay alive and unmoved until stop
 * succeeds; storage capacity is 1..32767 bytes. A start resets diagnostics.
 * Hardware bytes received before start may still arrive; framing is the
 * application's responsibility. A start does not flush the hardware FIFO.
 */
int nrf54l15_uart_rx_start(struct nrf54l15_uart_rx *rx, const struct device *dev,
	uint8_t *storage, uint32_t capacity);

/* Nonblocking: returns 0..capacity bytes, or negative errno. Capacity is 1..256
 * so copying under irq_lock remains bounded. Caller output must not overlap
 * state/storage. Errors leave output unchanged. Overflow or UART line errors
 * latch -EOVERFLOW and discard subsequent input until resync is acknowledged.
 */
int nrf54l15_uart_rx_read(struct nrf54l15_uart_rx *rx, uint8_t *output, uint32_t capacity);

/* Discard software backlog, acknowledge the gap and resume buffering. Does
 * not establish a wire/message boundary or clear a backend fault. Reset the
 * application parser and find its next delimiter/header before trusting data.
 */
int nrf54l15_uart_rx_resync(struct nrf54l15_uart_rx *rx);

/* Disable RX/error notifications, detach callback, and discard software data.
 * Hardware reception/power-down is SDK-dependent. Retain all storage and retry
 * on failure. Successful stop permits reuse/freeing state and a later start.
 * Diagnostics are retained until the next start; stopping an unbound object
 * is a no-op. A stopped UART may still contain a byte on restart.
 */
int nrf54l15_uart_rx_stop(struct nrf54l15_uart_rx *rx);

/* Coherent snapshot. Counters saturate at UINT32_MAX; hardware-lost bytes are
 * not countable from UART error flags and are absent from received/discarded.
 */
int nrf54l15_uart_rx_get_stats(const struct nrf54l15_uart_rx *rx,
	struct nrf54l15_uart_rx_stats *stats);

#ifdef __cplusplus
}
#endif
#endif
