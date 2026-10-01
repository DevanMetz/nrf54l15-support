/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <nrf54l15/spi.h>

int nrf54l15_spi_init(struct nrf54l15_spi *spi, const struct spi_dt_spec *spec)
{
	if (!spi || !spec || !spec->bus) {
		return -EINVAL;
	}
	if (spi->ready) {
		return -EALREADY;
	}
	const uint32_t allowed = SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA |
		SPI_TRANSFER_LSB | SPI_CS_ACTIVE_HIGH;
	if (SPI_WORD_SIZE_GET(spec->config.operation) != 8 ||
	    ((uint32_t)spec->config.operation & ~allowed) || spec->config.word_delay) {
		return -ENOTSUP;
	}
	if (spec->config.frequency < 125000 || spec->config.frequency > 32000000) {
		return -EINVAL;
	}
	if (spec->config.cs.cs_is_gpio) {
		if (!spec->config.cs.gpio.port) {
			return -EINVAL;
		}
		bool gpio_active_low = (spec->config.cs.gpio.dt_flags & GPIO_ACTIVE_LOW) != 0;
		bool spi_active_high = (spec->config.operation & SPI_CS_ACTIVE_HIGH) != 0;
		if (gpio_active_low == spi_active_high) {
			return -EINVAL;
		}
	} else if (spec->config.cs.setup_ns || spec->config.cs.hold_ns) {
		return -ENOTSUP;
	}
	if (!spi_is_ready_dt(spec)) {
		return -ENODEV;
	}
	spi->spec = *spec;
	spi->ready = true;
	return 0;
}

int nrf54l15_spi_transfer(struct nrf54l15_spi *spi, const void *tx, void *rx,
	size_t length, uint8_t dummy)
{
	if (!spi || !length) {
		return -EINVAL;
	}
	if (!spi->ready) {
		return -ENODEV;
	}
	if (length > NRF54L15_SPI_MAX_TRANSFER) {
		return -EMSGSIZE;
	}
	if (tx) {
		memcpy(spi->tx.bytes, tx, length);
	} else {
		memset(spi->tx.bytes, dummy, length);
	}
	struct spi_buf tx_buffer = {.buf = spi->tx.bytes, .len = length};
	struct spi_buf rx_buffer = {.buf = spi->rx.bytes, .len = length};
	struct spi_buf_set tx_set = {.buffers = &tx_buffer, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &rx_buffer, .count = 1};
	int err = spi_transceive_dt(&spi->spec, &tx_set, rx ? &rx_set : NULL);
	if (!err && rx) {
		memcpy(rx, spi->rx.bytes, length);
	}
	return err;
}
