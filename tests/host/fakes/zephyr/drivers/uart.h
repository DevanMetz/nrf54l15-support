/* Test doubles only. Firmware uses the real Zephyr UART API. */
#ifndef TEST_ZEPHYR_UART_H
#define TEST_ZEPHYR_UART_H
#include <stdint.h>
#include <zephyr/device.h>
#define UART_ERROR_OVERRUN (1U << 0)
#define UART_ERROR_PARITY (1U << 1)
#define UART_ERROR_FRAMING (1U << 2)
#define UART_BREAK (1U << 3)
typedef void (*uart_irq_callback_user_data_t)(const struct device *, void *);
int uart_irq_callback_user_data_set(const struct device *, uart_irq_callback_user_data_t, void *);
void uart_irq_rx_enable(const struct device *);
void uart_irq_rx_disable(const struct device *);
void uart_irq_err_enable(const struct device *);
void uart_irq_err_disable(const struct device *);
int uart_irq_update(const struct device *);
int uart_irq_rx_ready(const struct device *);
int uart_err_check(const struct device *);
int uart_fifo_read(const struct device *, uint8_t *, int);
#endif
