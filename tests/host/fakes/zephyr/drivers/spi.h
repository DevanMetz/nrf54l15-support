/* Test doubles only. Firmware uses the real Zephyr SPI API. */
#ifndef TEST_ZEPHYR_SPI_H
#define TEST_ZEPHYR_SPI_H
#include <stddef.h>
#include <zephyr/drivers/gpio.h>
#define SPI_OP_MODE_MASTER 0U
#define SPI_OP_MODE_SLAVE (1U << 0)
#define SPI_MODE_CPOL (1U << 1)
#define SPI_MODE_CPHA (1U << 2)
#define SPI_MODE_LOOP (1U << 3)
#define SPI_TRANSFER_MSB 0U
#define SPI_TRANSFER_LSB (1U << 4)
#define SPI_WORD_SET(bits) ((bits) << 5)
#define SPI_WORD_SIZE_GET(operation) (((operation) >> 5) & 0x3fU)
#define SPI_HALF_DUPLEX (1U << 11)
#define SPI_HOLD_ON_CS (1U << 12)
#define SPI_LOCK_ON (1U << 13)
#define SPI_CS_ACTIVE_HIGH (1U << 14)
#define SPI_FRAME_FORMAT_TI (1U << 15)
#define SPI_LINES_DUAL (1U << 16)
#define SPI_LINES_QUAD (2U << 16)
typedef uint32_t spi_operation_t;
struct spi_cs_control {
	union {
		struct { struct gpio_dt_spec gpio; uint32_t delay; };
		struct { uint32_t setup_ns; uint32_t hold_ns; };
	};
	bool cs_is_gpio;
};
struct spi_config {
	uint32_t frequency;
	spi_operation_t operation;
	uint16_t slave;
	struct spi_cs_control cs;
	uint16_t word_delay;
};
struct spi_dt_spec { const struct device *bus; struct spi_config config; };
struct spi_buf { void *buf; size_t len; };
struct spi_buf_set { const struct spi_buf *buffers; size_t count; };
static inline bool spi_is_ready_dt(const struct spi_dt_spec *spec)
{
	return device_is_ready(spec->bus) &&
		(!spec->config.cs.cs_is_gpio || gpio_is_ready_dt(&spec->config.cs.gpio));
}
int spi_transceive_dt(const struct spi_dt_spec *spec,
	const struct spi_buf_set *tx, const struct spi_buf_set *rx);
#endif
