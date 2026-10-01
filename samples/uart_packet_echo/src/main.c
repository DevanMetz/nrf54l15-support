/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <nrf54l15/packet.h>
#include <nrf54l15/uart_rx.h>

BUILD_ASSERT(!DT_SAME_NODE(DT_NODELABEL(uart21), DT_CHOSEN(zephyr_console)),
	"Packet UART must not own the console");
static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart21));
static struct nrf54l15_uart_rx receiver;
static struct nrf54l15_packet_decoder decoder;
static uint8_t storage[512], payload[NRF54L15_PACKET_MAX_PAYLOAD];
static uint8_t wire[NRF54L15_PACKET_MAX_WIRE];

static void increment(uint32_t *count)
{
	if (*count != UINT32_MAX) { ++*count; }
}

int main(void)
{
	(void)nrf54l15_packet_reset(&decoder);
	int err = nrf54l15_uart_rx_start(&receiver, uart, storage, sizeof(storage));
	if (err) {
		printk("# PACKET UART start failed: %d\n", err);
		return 0;
	}
	printk("# PACKET echo on D6/TX and D7/RX, 115200 8N1; payload 0..256 bytes\n");
	uint32_t echoed = 0, rejected = 0, gaps = 0;
	int64_t report_ms = k_uptime_get();
	for (;;) {
		uint8_t input[64];
		int count = nrf54l15_uart_rx_read(&receiver, input, sizeof(input));
		if (count == -EOVERFLOW) {
			(void)nrf54l15_packet_reset(&decoder);
			err = nrf54l15_uart_rx_resync(&receiver);
			if (err) { break; }
			increment(&gaps);
		} else if (count < 0) {
			err = count;
			break;
		} else {
			for (int i = 0; i < count; ++i) {
				size_t length;
				int result = nrf54l15_packet_feed(&decoder, input[i], payload, sizeof(payload), &length);
				if (result == 1) {
					size_t wire_length;
					err = nrf54l15_packet_encode(payload, length, wire, sizeof(wire), &wire_length);
					if (err) { break; }
					for (size_t j = 0; j < wire_length; ++j) { uart_poll_out(uart, wire[j]); }
					increment(&echoed);
				} else if (result < 0) {
					increment(&rejected);
				}
			}
			if (err) { break; }
		}
		int64_t now_ms = k_uptime_get();
		if (now_ms - report_ms >= 1000) {
			struct nrf54l15_uart_rx_stats stats;
			(void)nrf54l15_uart_rx_get_stats(&receiver, &stats);
			printk("PACKET,echoed=%u,rejected=%u,gaps=%u,discarded=%u\n",
				echoed, rejected, gaps, stats.discarded_bytes);
			report_ms = now_ms;
		}
		if (count <= 0) { k_sleep(K_MSEC(1)); }
	}
	int stop_err = nrf54l15_uart_rx_stop(&receiver);
	printk("# PACKET UART failed: error=%d stop=%d\n", err, stop_err);
	return 0;
}
