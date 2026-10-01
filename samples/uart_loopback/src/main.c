/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <nrf54l15/uart_rx.h>

BUILD_ASSERT(!DT_SAME_NODE(DT_NODELABEL(uart21), DT_CHOSEN(zephyr_console)),
	"Loopback UART must not own the console");
static struct nrf54l15_uart_rx receiver;
static uint8_t storage[128];
static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart21));

static int check_round(uint32_t round)
{
	uint8_t expected[64], received[64];
	for (size_t i = 0; i < sizeof(expected); ++i) {
		expected[i] = (uint8_t)(i * 37U + round);
		uart_poll_out(uart, expected[i]);
	}
	size_t used = 0;
	int64_t began_ms = k_uptime_get();
	while (used < sizeof(received)) {
		int count = nrf54l15_uart_rx_read(&receiver, received + used, sizeof(received) - used);
		if (count < 0) { return count; }
		used += (size_t)count;
		if (used == sizeof(received)) { break; }
		if (k_uptime_get() - began_ms >= 500) { return -ETIMEDOUT; }
		k_sleep(K_MSEC(1));
	}
	return memcmp(expected, received, sizeof(expected)) ? -EBADMSG : 0;
}

int main(void)
{
	int err = nrf54l15_uart_rx_start(&receiver, uart, storage, sizeof(storage));
	if (err) {
		printk("# UART start failed: %d\n", err);
		return 0;
	}
	printk("# UART wire loopback: D6/TX to D7/RX; 115200 8N1\n");
	k_sleep(K_MSEC(1000));
	err = nrf54l15_uart_rx_resync(&receiver);
	for (uint32_t round = 0; !err; ++round) {
		err = check_round(round);
		if (err) { break; }
		/* Deliberately exceed the 128-byte queue without consuming input. */
		for (unsigned int i = 0; i < 192; ++i) { uart_poll_out(uart, (uint8_t)i); }
		k_sleep(K_MSEC(20));
		uint8_t byte;
		int read_result = nrf54l15_uart_rx_read(&receiver, &byte, 1);
		if (read_result != -EOVERFLOW) {
			err = read_result < 0 ? read_result : -EIO;
			break;
		}
		err = nrf54l15_uart_rx_resync(&receiver);
		if (!err) { err = check_round(round + 1); }
		if (err) { break; }
		struct nrf54l15_uart_rx_stats stats;
		(void)nrf54l15_uart_rx_get_stats(&receiver, &stats);
		if (stats.uart_error_events) { err = -EIO; break; }
		printk("UART,%u,PASS,received=%u,discarded=%u,overflows=%u\n",
			round, stats.received_bytes, stats.discarded_bytes, stats.overflow_events);
		k_sleep(K_MSEC(1000));
	}
	int stop_err = nrf54l15_uart_rx_stop(&receiver);
	struct nrf54l15_uart_rx_stats stats;
	(void)nrf54l15_uart_rx_get_stats(&receiver, &stats);
	printk("# UART failed: error=%d stop=%d irq_fault=%d line_errors=0x%x\n",
		err, stop_err, stats.fault, stats.uart_error_mask);
	return 0;
}
