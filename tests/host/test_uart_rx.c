/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <nrf54l15/packet.h>
#include <nrf54l15/uart_rx.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static struct device dev = {.ready = true};
static struct nrf54l15_uart_rx receiver;
static uint8_t storage[NRF54L15_UART_RX_MAX_CAPACITY];
static uart_irq_callback_user_data_t callback;
static void *callback_data;
static bool rx_irq, err_irq, in_isr, interrupt_on_enable, pending_on_unlock;
static unsigned int lock_depth, installs, detaches, fifo_calls, disable_calls;
static int install_error, detach_error, update_result, ready_result, error_result, fifo_error;
static unsigned int line_errors, error_after_fifo;
static int fifo_quantum;
static uint8_t hardware[4096];
static size_t hw_head, hw_count;

static void invoke_irq(void)
{
	CHECK(callback && !lock_depth && !in_isr);
	in_isr = true;
	callback(&dev, callback_data);
	in_isr = false;
}

unsigned int irq_lock(void) { return lock_depth++; }
void irq_unlock(unsigned int key)
{
	CHECK(lock_depth == key + 1);
	lock_depth = key;
	if (!lock_depth && pending_on_unlock) {
		pending_on_unlock = false;
		invoke_irq();
	}
}

void test_ring_guard(int producer)
{
	CHECK(producer ? in_isr : lock_depth > 0);
}

static void reset_backend(void)
{
	receiver = (struct nrf54l15_uart_rx){0};
	dev.ready = true;
	callback = NULL;
	callback_data = NULL;
	rx_irq = err_irq = in_isr = interrupt_on_enable = pending_on_unlock = false;
	lock_depth = installs = detaches = fifo_calls = disable_calls = 0;
	install_error = detach_error = error_result = fifo_error = 0;
	update_result = 1;
	ready_result = INT_MAX;
	line_errors = error_after_fifo = 0;
	fifo_quantum = 1; /* NCS nrfx IRQ backend returns one byte per completion. */
	hw_head = hw_count = 0;
}

int uart_irq_callback_user_data_set(const struct device *device,
	uart_irq_callback_user_data_t cb, void *data)
{
	CHECK(device == &dev && lock_depth && !in_isr && !rx_irq && !err_irq);
	if (cb) {
		++installs;
		if (install_error) { return install_error; }
	} else {
		++detaches;
		if (detach_error) { return detach_error; }
	}
	callback = cb;
	callback_data = data;
	return 0;
}
void uart_irq_rx_enable(const struct device *device)
{
	CHECK(device == &dev && !lock_depth);
	rx_irq = true;
	if (interrupt_on_enable) { invoke_irq(); }
}
void uart_irq_err_enable(const struct device *device)
{
	CHECK(device == &dev && !lock_depth);
	err_irq = true;
	if (interrupt_on_enable) { invoke_irq(); }
}
void uart_irq_rx_disable(const struct device *device)
{
	CHECK(device == &dev);
	rx_irq = false;
	++disable_calls;
}
void uart_irq_err_disable(const struct device *device) { CHECK(device == &dev); err_irq = false; }
int uart_irq_update(const struct device *device) { CHECK(device == &dev && in_isr); return update_result; }
int uart_irq_rx_ready(const struct device *device)
{
	CHECK(device == &dev && in_isr);
	return ready_result == INT_MAX ? hw_count != 0 : ready_result;
}
int uart_err_check(const struct device *device)
{
	CHECK(device == &dev && in_isr);
	if (error_result) { return error_result; }
	int result = (int)line_errors;
	line_errors = 0;
	return result;
}
int uart_fifo_read(const struct device *device, uint8_t *data, int capacity)
{
	CHECK(device == &dev && in_isr && capacity == 16);
	++fifo_calls;
	if (fifo_error) { return fifo_error; }
	int count = hw_count < (size_t)capacity ? (int)hw_count : capacity;
	if (count > fifo_quantum) { count = fifo_quantum; }
	for (int i = 0; i < count; ++i) { data[i] = hardware[(hw_head + (size_t)i) % sizeof(hardware)]; }
	hw_head = (hw_head + (size_t)count) % sizeof(hardware);
	hw_count -= (size_t)count;
	line_errors |= error_after_fifo;
	error_after_fifo = 0;
	return count;
}

static void enqueue(const uint8_t *data, size_t count)
{
	CHECK(count <= sizeof(hardware) - hw_count);
	for (size_t i = 0; i < count; ++i) { hardware[(hw_head + hw_count + i) % sizeof(hardware)] = data[i]; }
	hw_count += count;
}

static void pump(void)
{
	unsigned int iterations = 0;
	while (rx_irq && hw_count) {
		CHECK(++iterations < 5000);
		invoke_irq();
	}
}

static void receive(const uint8_t *data, size_t count) { enqueue(data, count); pump(); }

static struct nrf54l15_uart_rx_stats stats(void)
{
	struct nrf54l15_uart_rx_stats result;
	CHECK(nrf54l15_uart_rx_get_stats(&receiver, &result) == 0 && !lock_depth);
	return result;
}

static void start(uint32_t capacity)
{
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, capacity) == 0);
	CHECK(callback_data == &receiver && stats().running && rx_irq && err_irq);
}

static void test_validation_and_registration(void)
{
	reset_backend();
	uint8_t output[4];
	CHECK(nrf54l15_uart_rx_start(NULL, &dev, storage, 4) == -EINVAL);
	CHECK(nrf54l15_uart_rx_start(&receiver, NULL, storage, 4) == -EINVAL);
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, NULL, 4) == -EINVAL);
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, 0) == -EINVAL);
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, 32768) == -EINVAL);
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, UINT32_MAX) == -EINVAL);
	dev.ready = false;
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, 4) == -ENODEV);
	CHECK(installs == 0 && disable_calls == 0);
	dev.ready = true;
	CHECK(nrf54l15_uart_rx_read(&receiver, output, 4) == -ENODEV);
	CHECK(nrf54l15_uart_rx_read(NULL, output, 4) == -EINVAL);
	CHECK(nrf54l15_uart_rx_read(&receiver, NULL, 4) == -EINVAL);
	CHECK(nrf54l15_uart_rx_read(&receiver, output, 0) == -EINVAL);
	CHECK(nrf54l15_uart_rx_read(&receiver, output, 257) == -EINVAL);
	CHECK(nrf54l15_uart_rx_resync(NULL) == -EINVAL);
	CHECK(nrf54l15_uart_rx_resync(&receiver) == -ENODEV);
	CHECK(nrf54l15_uart_rx_get_stats(NULL, &receiver.stats) == -EINVAL);
	CHECK(nrf54l15_uart_rx_get_stats(&receiver, NULL) == -EINVAL);
	CHECK(nrf54l15_uart_rx_stop(NULL) == -EINVAL);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0 && detaches == 0);
	install_error = -ENOTSUP;
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, 4) == -ENOTSUP);
	CHECK(!receiver.bound && !callback && !rx_irq && !err_irq);
	install_error = 0;
	start(4);
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, 4) == -EALREADY && installs == 2);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0 && !receiver.bound && !callback);
	start(NRF54L15_UART_RX_MAX_CAPACITY);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
}

static void test_order_and_wrap(void)
{
	reset_backend();
	start(17);
	uint8_t input[5], output[8];
	for (unsigned int round = 0; round < 1000; ++round) {
		for (size_t i = 0; i < sizeof(input); ++i) { input[i] = (uint8_t)(round * 5 + i); }
		receive(input, sizeof(input));
		memset(output, 0xa7, sizeof(output));
		CHECK(nrf54l15_uart_rx_read(&receiver, output, 3) == 3);
		CHECK(nrf54l15_uart_rx_read(&receiver, output + 3, 5) == 2);
		CHECK(memcmp(input, output, 5) == 0 && output[5] == 0xa7 && output[7] == 0xa7);
		CHECK(nrf54l15_uart_rx_read(&receiver, output, 8) == 0);
	}
	CHECK(stats().received_bytes == 5000 && stats().discarded_bytes == 0);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
	/* Also exercise backends returning multi-byte FIFO chunks. */
	reset_backend();
	start(64);
	fifo_quantum = 16;
	uint8_t block[48], result[48];
	for (size_t i = 0; i < sizeof(block); ++i) { block[i] = (uint8_t)i; }
	receive(block, sizeof(block));
	CHECK(fifo_calls == 4); /* Three full reads, then a short/zero read. */
	CHECK(nrf54l15_uart_rx_read(&receiver, result, sizeof(result)) == 48);
	CHECK(memcmp(block, result, sizeof(block)) == 0);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
}

static void test_gap_and_resync(void)
{
	reset_backend();
	start(4);
	const uint8_t input[] = {1, 2, 3, 4, 5, 6};
	uint8_t output[8];
	memset(output, 0xc4, sizeof(output));
	receive(input, 6);
	CHECK(stats().received_bytes == 6 && stats().discarded_bytes == 2);
	CHECK(stats().overflow_events == 1 && stats().gap_pending);
	CHECK(nrf54l15_uart_rx_read(&receiver, output, 8) == -EOVERFLOW && output[0] == 0xc4);
	receive(input, 3);
	CHECK(stats().discarded_bytes == 5 && stats().overflow_events == 1);
	CHECK(nrf54l15_uart_rx_resync(&receiver) == 0);
	CHECK(stats().discarded_bytes == 9 && !stats().gap_pending);
	CHECK(nrf54l15_uart_rx_read(&receiver, output, 8) == 0);
	receive(input, 3);
	CHECK(nrf54l15_uart_rx_read(&receiver, output, 8) == 3 && memcmp(input, output, 3) == 0);
	receive(input, 6);
	CHECK(stats().overflow_events == 2);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
	CHECK(!stats().running && !stats().gap_pending && stats().discarded_bytes == 15);
	start(1);
	CHECK(stats().received_bytes == 0 && stats().overflow_events == 0);
	receive(input, 1);
	CHECK(nrf54l15_uart_rx_read(&receiver, output, 1) == 1 && output[0] == 1);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
}

static void test_line_errors(void)
{
	const unsigned int errors[] = {UART_ERROR_OVERRUN, UART_ERROR_PARITY,
		UART_ERROR_FRAMING, UART_BREAK, UART_ERROR_PARITY | UART_ERROR_FRAMING};
	for (size_t i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i) {
		reset_backend();
		start(8);
		uint8_t byte = 0x42, output = 0x91;
		receive(&byte, 1);
		line_errors = errors[i];
		invoke_irq(); /* Error-only interrupt with no received character. */
		CHECK(stats().uart_error_mask == errors[i] && stats().uart_error_events == 1);
		CHECK(stats().overflow_events == 0 && nrf54l15_uart_rx_read(&receiver, &output, 1) == -EOVERFLOW);
		CHECK(output == 0x91 && rx_irq && err_irq);
		receive(&byte, 1);
		CHECK(stats().discarded_bytes == 1);
		CHECK(nrf54l15_uart_rx_resync(&receiver) == 0 && stats().discarded_bytes == 2);
		error_after_fifo = UART_ERROR_PARITY;
		receive(&byte, 1);
		CHECK(stats().gap_pending && stats().uart_error_events == 2);
		CHECK(stats().uart_error_mask == (errors[i] | UART_ERROR_PARITY));
		CHECK(nrf54l15_uart_rx_read(&receiver, &output, 1) == -EOVERFLOW && output == 0x91);
		CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
	}
}

static void test_backend_faults(void)
{
	for (unsigned int stage = 0; stage < 5; ++stage) {
		reset_backend();
		start(8);
		uint8_t input = 1, output = 0x32;
		switch (stage) {
		case 0: update_result = -ENOSYS; break;
		case 1: error_result = -ENOSYS; break;
		case 2: ready_result = -ENOSYS; break;
		case 3: fifo_error = -ENOSYS; break;
		default: fifo_error = 17; break; /* Invalid count larger than scratch buffer. */
		}
		enqueue(&input, 1);
		invoke_irq();
		int expected = stage == 4 ? -EIO : -ENOSYS;
		CHECK(stats().fault == expected && !stats().running && !rx_irq && !err_irq);
		CHECK(nrf54l15_uart_rx_read(&receiver, &output, 1) == expected && output == 0x32);
		CHECK(nrf54l15_uart_rx_resync(&receiver) == expected);
		CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, 8) == -EALREADY);
		CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
	}
	reset_backend();
	start(8);
	update_result = 0;
	invoke_irq();
	CHECK(stats().fault == 0 && fifo_calls == 0);
	update_result = 1;
	ready_result = 1; /* Spurious ready with an empty FIFO must not spin. */
	invoke_irq();
	CHECK(fifo_calls == 1 && stats().received_bytes == 0);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
	/* Fault during the first enable cannot leave the second IRQ source armed. */
	reset_backend();
	error_result = -EIO;
	interrupt_on_enable = true;
	CHECK(nrf54l15_uart_rx_start(&receiver, &dev, storage, 8) == 0);
	CHECK(stats().fault == -EIO && !rx_irq && !err_irq);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
}

static void test_stop_and_snapshot(void)
{
	reset_backend();
	start(8);
	const uint8_t data[] = {11, 22, 33};
	receive(data, 3);
	detach_error = -EIO;
	CHECK(nrf54l15_uart_rx_stop(&receiver) == -EIO);
	CHECK(receiver.bound && callback && !stats().running && !rx_irq && !err_irq);
	CHECK(stats().discarded_bytes == 0);
	invoke_irq(); /* A late callback remains safe while storage is retained. */
	CHECK(nrf54l15_uart_rx_resync(&receiver) == -EIO);
	detach_error = 0;
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0 && !callback && !receiver.bound);
	CHECK(stats().discarded_bytes == 3 && stats().fault == -EIO);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0 && detaches == 2);
	start(8);
	CHECK(stats().fault == 0 && stats().discarded_bytes == 0);
	/* Simulate arrival immediately when read releases IRQ exclusion. */
	enqueue(data, 1);
	pending_on_unlock = true;
	uint8_t output = 0x74;
	CHECK(nrf54l15_uart_rx_read(&receiver, &output, 1) == 0 && output == 0x74);
	CHECK(nrf54l15_uart_rx_read(&receiver, &output, 1) == 1 && output == 11);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
}

static void test_saturation(void)
{
	reset_backend();
	start(1);
	receiver.stats.received_bytes = UINT32_MAX - 1;
	receiver.stats.discarded_bytes = UINT32_MAX - 1;
	receiver.stats.overflow_events = UINT32_MAX;
	receiver.stats.uart_error_events = UINT32_MAX;
	const uint8_t input[] = {1, 2, 3};
	receive(input, 3);
	line_errors = UART_ERROR_OVERRUN;
	invoke_irq();
	CHECK(nrf54l15_uart_rx_resync(&receiver) == 0);
	CHECK(stats().received_bytes == UINT32_MAX && stats().discarded_bytes == UINT32_MAX);
	CHECK(stats().overflow_events == UINT32_MAX && stats().uart_error_events == UINT32_MAX);
	CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
}

static int packet_byte(struct nrf54l15_packet_decoder *decoder, uint8_t byte,
	uint8_t *payload, size_t *length)
{
	receive(&byte, 1);
	uint8_t input = 0;
	CHECK(nrf54l15_uart_rx_read(&receiver, &input, 1) == 1);
	return nrf54l15_packet_feed(decoder, input, payload, 32, length);
}

static void test_packet_recovery_after_uart_loss(void)
{
	uint8_t original[32], payload[32], wire[NRF54L15_PACKET_MAX_WIRE];
	for (size_t i = 0; i < sizeof(original); ++i) { original[i] = (uint8_t)(i * 37); }
	size_t wire_length = 0, length = SIZE_MAX;
	CHECK(nrf54l15_packet_encode(original, sizeof(original), wire, sizeof(wire), &wire_length) == 0);
	for (unsigned int line_error = 0; line_error < 2; ++line_error) {
		for (size_t cut = 1; cut < wire_length; ++cut) {
			reset_backend();
			start(8);
			struct nrf54l15_packet_decoder decoder;
			CHECK(nrf54l15_packet_reset(&decoder) == 0);
			for (size_t i = 0; i < cut; ++i) {
				CHECK(packet_byte(&decoder, wire[i], payload, &length) == 0);
			}
			if (line_error) {
				line_errors = UART_ERROR_FRAMING;
				invoke_irq();
			} else {
				const uint8_t overflow[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
				receive(overflow, sizeof(overflow));
			}
			uint8_t input = 0xa5;
			CHECK(nrf54l15_uart_rx_read(&receiver, &input, 1) == -EOVERFLOW && input == 0xa5);
			CHECK(nrf54l15_packet_reset(&decoder) == 0);
			CHECK(nrf54l15_uart_rx_resync(&receiver) == 0);
			for (size_t i = cut; i < wire_length; ++i) {
				CHECK(packet_byte(&decoder, wire[i], payload, &length) == 0);
			}
			for (size_t i = 0; i < wire_length; ++i) {
				CHECK(packet_byte(&decoder, wire[i], payload, &length) == (i == wire_length - 1));
			}
			CHECK(length == sizeof(original) && !memcmp(payload, original, length));
			CHECK(nrf54l15_uart_rx_stop(&receiver) == 0);
		}
	}
}

int main(void)
{
	test_validation_and_registration();
	test_order_and_wrap();
	test_gap_and_resync();
	test_line_errors();
	test_backend_faults();
	test_stop_and_snapshot();
	test_saturation();
	test_packet_recovery_after_uart_loss();
	puts("uart_rx tests passed");
	return 0;
}
