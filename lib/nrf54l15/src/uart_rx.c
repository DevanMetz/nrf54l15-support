/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <nrf54l15/uart_rx.h>

#if defined(CONFIG_SMP)
#error "uart_rx uses single-core IRQ exclusion; SMP is not supported"
#endif

static void add_count(uint32_t *count, uint32_t amount)
{
	*count = amount > UINT32_MAX - *count ? UINT32_MAX : *count + amount;
}

static void fault(struct nrf54l15_uart_rx *rx, int error)
{
	if (!rx->stats.fault) {
		rx->stats.fault = error;
	}
	rx->stats.running = false;
	uart_irq_rx_disable(rx->dev);
	uart_irq_err_disable(rx->dev);
}

static bool check_errors(struct nrf54l15_uart_rx *rx)
{
	int errors = uart_err_check(rx->dev);
	if (errors < 0) {
		fault(rx, errors);
		return false;
	}
	if (errors) {
		rx->stats.uart_error_mask |= (uint32_t)errors;
		add_count(&rx->stats.uart_error_events, 1);
		rx->stats.gap_pending = true;
	}
	return true;
}

static void on_irq(const struct device *dev, void *user_data)
{
	struct nrf54l15_uart_rx *rx = user_data;
	if (!rx->bound) {
		return;
	}
	if (rx->stats.fault) {
		/* A fault may race with the two interrupt-enable calls at startup. */
		fault(rx, rx->stats.fault);
		return;
	}
	int err = uart_irq_update(dev);
	if (err < 0) {
		fault(rx, err);
		return;
	}
	if (!err || !check_errors(rx)) {
		return;
	}
	err = uart_irq_rx_ready(dev);
	if (err < 0) {
		fault(rx, err);
		return;
	}
	if (!err) {
		return;
	}
	uint8_t bytes[16];
	for (;;) {
		int count = uart_fifo_read(dev, bytes, sizeof(bytes));
		if (count < 0 || count > (int)sizeof(bytes)) {
			fault(rx, count < 0 ? count : -EIO);
			return;
		}
		if (!count) {
			return;
		}
		add_count(&rx->stats.received_bytes, (uint32_t)count);
		if (!check_errors(rx)) {
			add_count(&rx->stats.discarded_bytes, (uint32_t)count);
			return;
		}
		uint32_t saved = 0;
		if (!rx->stats.gap_pending) {
			saved = ring_buf_put(&rx->ring, bytes, (uint32_t)count);
			if (saved != (uint32_t)count) {
				add_count(&rx->stats.overflow_events, 1);
				rx->stats.gap_pending = true;
			}
		}
		add_count(&rx->stats.discarded_bytes, (uint32_t)count - saved);
		if (count < (int)sizeof(bytes)) {
			return; /* Zephyr requires draining until a short FIFO read. */
		}
	}
}

int nrf54l15_uart_rx_start(struct nrf54l15_uart_rx *rx, const struct device *dev,
	uint8_t *storage, uint32_t capacity)
{
	if (!rx || !dev || !storage || !capacity || capacity > NRF54L15_UART_RX_MAX_CAPACITY) {
		return -EINVAL;
	}
	if (rx->bound) {
		return -EALREADY;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	uart_irq_rx_disable(dev);
	uart_irq_err_disable(dev);
	*rx = (struct nrf54l15_uart_rx){.dev = dev};
	ring_buf_init(&rx->ring, capacity, storage);
	unsigned int key = irq_lock();
	int err = uart_irq_callback_user_data_set(dev, on_irq, rx);
	if (!err) {
		rx->bound = true;
		rx->stats.running = true;
	}
	irq_unlock(key);
	if (!err) {
		uart_irq_err_enable(dev);
		uart_irq_rx_enable(dev);
	}
	return err;
}

int nrf54l15_uart_rx_read(struct nrf54l15_uart_rx *rx, uint8_t *output, uint32_t capacity)
{
	if (!rx || !output || !capacity || capacity > NRF54L15_UART_RX_MAX_READ) {
		return -EINVAL;
	}
	unsigned int key = irq_lock();
	int result;
	if (!rx->bound) {
		result = -ENODEV;
	} else if (rx->stats.fault) {
		result = rx->stats.fault;
	} else if (rx->stats.gap_pending) {
		result = -EOVERFLOW;
	} else {
		result = (int)ring_buf_get(&rx->ring, output, capacity);
	}
	irq_unlock(key);
	return result;
}

int nrf54l15_uart_rx_resync(struct nrf54l15_uart_rx *rx)
{
	if (!rx) {
		return -EINVAL;
	}
	unsigned int key = irq_lock();
	int err = !rx->bound ? -ENODEV : rx->stats.fault;
	if (!err) {
		add_count(&rx->stats.discarded_bytes, ring_buf_size_get(&rx->ring));
		ring_buf_reset(&rx->ring);
		rx->stats.gap_pending = false;
	}
	irq_unlock(key);
	return err;
}

int nrf54l15_uart_rx_stop(struct nrf54l15_uart_rx *rx)
{
	if (!rx) {
		return -EINVAL;
	}
	if (!rx->bound) {
		return 0;
	}
	uart_irq_rx_disable(rx->dev);
	uart_irq_err_disable(rx->dev);
	unsigned int key = irq_lock();
	int err = uart_irq_callback_user_data_set(rx->dev, NULL, NULL);
	rx->stats.running = false;
	if (err) {
		if (!rx->stats.fault) { rx->stats.fault = err; }
	} else {
		rx->bound = false;
		add_count(&rx->stats.discarded_bytes, ring_buf_size_get(&rx->ring));
		ring_buf_reset(&rx->ring);
		rx->stats.gap_pending = false;
	}
	irq_unlock(key);
	return err;
}

int nrf54l15_uart_rx_get_stats(const struct nrf54l15_uart_rx *rx,
	struct nrf54l15_uart_rx_stats *stats)
{
	if (!rx || !stats) {
		return -EINVAL;
	}
	unsigned int key = irq_lock();
	*stats = rx->stats;
	irq_unlock(key);
	return 0;
}
