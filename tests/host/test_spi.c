/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/spi.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static struct device bus = {.ready = true}, gpio = {.ready = true};
static struct nrf54l15_spi spi;
static unsigned int transfers;
static int transfer_error;
static size_t fail_after, captured_length;
static bool received;
static uint8_t captured[NRF54L15_SPI_MAX_TRANSFER];
static const struct spi_dt_spec *last_spec;
static struct spi_config captured_config;
static const void *caller_tx, *caller_rx;

static struct spi_dt_spec default_spec(void)
{
	return (struct spi_dt_spec){
		.bus = &bus,
		.config = {
			.frequency = 1000000, .operation = SPI_WORD_SET(8),
			.cs = {.gpio = {.port = &gpio, .pin = 4, .dt_flags = GPIO_ACTIVE_LOW},
				.delay = 2, .cs_is_gpio = true},
		},
	};
}

static void reset_backend(void)
{
	/* Reset only between independent test fixtures, not a live firmware API. */
	spi = (struct nrf54l15_spi){0};
	bus.ready = gpio.ready = true;
	transfers = 0;
	transfer_error = 0;
	fail_after = 0;
	captured_length = 0;
	received = false;
	last_spec = NULL;
	caller_tx = caller_rx = NULL;
}

int spi_transceive_dt(const struct spi_dt_spec *spec,
	const struct spi_buf_set *tx, const struct spi_buf_set *rx)
{
	CHECK(spec->bus == &bus && tx && tx->count == 1 && tx->buffers);
	const struct spi_buf *out = &tx->buffers[0];
	CHECK(out->buf && out->len > 0 && out->len <= NRF54L15_SPI_MAX_TRANSFER);
	CHECK((uintptr_t)out->buf % 4 == 0 && out->buf != caller_tx);
	++transfers;
	last_spec = spec;
	captured_config = spec->config;
	captured_length = out->len;
	memcpy(captured, out->buf, out->len);
	received = rx != NULL;
	if (rx) {
		CHECK(rx->count == 1 && rx->buffers && rx->buffers[0].len == out->len);
		void *in = rx->buffers[0].buf;
		CHECK(in && (uintptr_t)in % 4 == 0 && in != caller_rx && in != out->buf);
		/* Model a full-duplex device. Inject failures after partial DMA writes. */
		size_t copied = transfer_error && fail_after < out->len ? fail_after : out->len;
		for (size_t i = 0; i < copied; ++i) {
			((uint8_t *)in)[i] = captured[i] ^ 0x5aU;
		}
	}
	return transfer_error;
}

static void init_default(void)
{
	struct spi_dt_spec spec = default_spec();
	CHECK(nrf54l15_spi_init(&spi, &spec) == 0 && spi.ready);
	CHECK(transfers == 0);
}

static int transfer(const void *tx, void *rx, size_t length, uint8_t dummy)
{
	caller_tx = tx;
	caller_rx = rx;
	return nrf54l15_spi_transfer(&spi, tx, rx, length, dummy);
}

static void test_validation(void)
{
	reset_backend();
	struct spi_dt_spec spec = default_spec();
	CHECK(nrf54l15_spi_init(NULL, &spec) == -EINVAL);
	CHECK(nrf54l15_spi_init(&spi, NULL) == -EINVAL);
	spec.bus = NULL;
	CHECK(nrf54l15_spi_init(&spi, &spec) == -EINVAL);
	spec = default_spec();
	const uint32_t flags[] = {SPI_OP_MODE_SLAVE, SPI_MODE_LOOP, SPI_HALF_DUPLEX,
		SPI_HOLD_ON_CS, SPI_LOCK_ON, SPI_FRAME_FORMAT_TI, SPI_LINES_DUAL,
		SPI_LINES_QUAD, 1U << 31};
	for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); ++i) {
		spec.config.operation = SPI_WORD_SET(8) | flags[i];
		CHECK(nrf54l15_spi_init(&spi, &spec) == -ENOTSUP && !spi.ready);
	}
	for (unsigned int bits = 0; bits <= 63; ++bits) {
		if (bits != 8) {
			spec.config.operation = SPI_WORD_SET(bits);
			CHECK(nrf54l15_spi_init(&spi, &spec) == -ENOTSUP);
		}
	}
	spec = default_spec();
	spec.config.word_delay = 1;
	CHECK(nrf54l15_spi_init(&spi, &spec) == -ENOTSUP);
	spec = default_spec();
	spec.config.cs = (struct spi_cs_control){.setup_ns = 1};
	CHECK(nrf54l15_spi_init(&spi, &spec) == -ENOTSUP);
	spec.config.cs = (struct spi_cs_control){.hold_ns = 1};
	CHECK(nrf54l15_spi_init(&spi, &spec) == -ENOTSUP);
	spec = default_spec();
	const uint32_t frequencies[] = {0, 124999, 32000001, UINT32_MAX};
	for (size_t i = 0; i < sizeof(frequencies) / sizeof(frequencies[0]); ++i) {
		spec.config.frequency = frequencies[i];
		CHECK(nrf54l15_spi_init(&spi, &spec) == -EINVAL);
	}
	spec = default_spec();
	spec.config.cs.gpio.port = NULL;
	CHECK(nrf54l15_spi_init(&spi, &spec) == -EINVAL);
	spec = default_spec();
	spec.config.cs.gpio.dt_flags = GPIO_ACTIVE_HIGH;
	CHECK(nrf54l15_spi_init(&spi, &spec) == -EINVAL);
	spec = default_spec();
	spec.config.operation |= SPI_CS_ACTIVE_HIGH;
	CHECK(nrf54l15_spi_init(&spi, &spec) == -EINVAL);
	spec = default_spec();
	bus.ready = false;
	CHECK(nrf54l15_spi_init(&spi, &spec) == -ENODEV);
	bus.ready = true;
	gpio.ready = false;
	CHECK(nrf54l15_spi_init(&spi, &spec) == -ENODEV);
	CHECK(!spi.ready && transfers == 0);
	CHECK(nrf54l15_spi_transfer(NULL, NULL, NULL, 1, 0xff) == -EINVAL);
	CHECK(transfer(NULL, NULL, 1, 0xff) == -ENODEV);
	gpio.ready = true;
	init_default();
	CHECK(transfer(NULL, NULL, 0, 0xff) == -EINVAL);
	CHECK(transfer(NULL, NULL, NRF54L15_SPI_MAX_TRANSFER + 1, 0xff) == -EMSGSIZE);
	CHECK(transfer(NULL, NULL, SIZE_MAX, 0xff) == -EMSGSIZE && transfers == 0);
}

static void test_configurations(void)
{
	const uint32_t rates[] = {125000, 250000, 1000000, 32000000};
	for (unsigned int mode = 0; mode < 4; ++mode) {
		for (unsigned int order = 0; order < 2; ++order) {
			for (unsigned int cs = 0; cs < 3; ++cs) {
				reset_backend();
				struct spi_dt_spec spec = default_spec();
				spec.config.frequency = rates[mode];
				spec.config.operation |= (mode & 1 ? SPI_MODE_CPHA : 0) |
					(mode & 2 ? SPI_MODE_CPOL : 0) | (order ? SPI_TRANSFER_LSB : 0);
				if (cs == 1) {
					spec.config.cs.gpio.dt_flags = GPIO_ACTIVE_HIGH;
					spec.config.operation |= SPI_CS_ACTIVE_HIGH;
				} else if (cs == 2) {
					spec.config.cs = (struct spi_cs_control){0};
					gpio.ready = false; /* No CS must not need a GPIO device. */
				}
				spi_operation_t operation = spec.config.operation;
				CHECK(nrf54l15_spi_init(&spi, &spec) == 0);
				spec.config.frequency = 2000000;
				spec.config.operation = SPI_WORD_SET(16);
				CHECK(nrf54l15_spi_init(&spi, &spec) == -EALREADY);
				CHECK(transfer(NULL, NULL, 1, 0xff) == 0);
				CHECK(last_spec == &spi.spec && captured_config.frequency == rates[mode]);
				CHECK(captured_config.operation == operation);
				CHECK(captured_config.cs.cs_is_gpio == (cs != 2));
				if (cs != 2) {
					CHECK(captured_config.cs.gpio.pin == 4 && captured_config.cs.delay == 2);
				}
			}
		}
	}
}

static void test_buffers(void)
{
	reset_backend();
	init_default();
	uint8_t tx[NRF54L15_SPI_MAX_TRANSFER + 2];
	uint8_t rx[NRF54L15_SPI_MAX_TRANSFER + 2];
	for (size_t i = 0; i < sizeof(tx); ++i) { tx[i] = (uint8_t)(i * 73U + 0xa5U); }
	for (size_t length = 1; length <= NRF54L15_SPI_MAX_TRANSFER; ++length) {
		memset(rx, 0xa9, sizeof(rx));
		CHECK(transfer(tx + 1, rx + 1, length, 0xff) == 0);
		CHECK(received && captured_length == length && memcmp(captured, tx + 1, length) == 0);
		CHECK(rx[0] == 0xa9);
		for (size_t i = 0; i < length; ++i) { CHECK(rx[i + 1] == (uint8_t)(tx[i + 1] ^ 0x5aU)); }
		for (size_t i = length + 1; i < sizeof(rx); ++i) { CHECK(rx[i] == 0xa9); }
	}
	const uint8_t command[] = {0x9f, 0, 0, 0};
	CHECK(transfer(command, rx, sizeof(command), 0xff) == 0);
	CHECK(memcmp(captured, command, sizeof(command)) == 0);
	CHECK(transfer(command, NULL, sizeof(command), 0xff) == 0 && !received);
	for (unsigned int fill = 0; fill <= UINT8_MAX; ++fill) {
		CHECK(transfer(NULL, rx, NRF54L15_SPI_MAX_TRANSFER, (uint8_t)fill) == 0);
		for (size_t i = 0; i < NRF54L15_SPI_MAX_TRANSFER; ++i) {
			CHECK(captured[i] == fill && rx[i] == (uint8_t)(fill ^ 0x5aU));
		}
	}
	CHECK(transfer(NULL, NULL, 7, 0x33) == 0 && !received);
	CHECK(captured_length == 7 && captured[6] == 0x33);
}

static void test_overlaps_and_partial_failures(void)
{
	reset_backend();
	init_default();
	uint8_t bytes[NRF54L15_SPI_MAX_TRANSFER + 2];
	uint8_t original[sizeof(bytes)];
	for (size_t i = 0; i < sizeof(bytes); ++i) { original[i] = (uint8_t)(i * 37U); }
	for (size_t source = 0; source <= 1; ++source) {
		for (size_t destination = 0; destination <= 1; ++destination) {
			memcpy(bytes, original, sizeof(bytes));
			CHECK(transfer(bytes + source, bytes + destination, NRF54L15_SPI_MAX_TRANSFER, 0) == 0);
			CHECK(memcmp(captured, original + source, NRF54L15_SPI_MAX_TRANSFER) == 0);
			for (size_t i = 0; i < NRF54L15_SPI_MAX_TRANSFER; ++i) {
				CHECK(bytes[destination + i] == (uint8_t)(original[source + i] ^ 0x5aU));
			}
		}
	}
	const int errors[] = {-EIO, -ETIMEDOUT, -EBUSY, -ENOMEM};
	const size_t partials[] = {0, 1, 255, NRF54L15_SPI_MAX_TRANSFER};
	for (size_t err = 0; err < sizeof(errors) / sizeof(errors[0]); ++err) {
		transfer_error = errors[err];
		for (size_t partial = 0; partial < sizeof(partials) / sizeof(partials[0]); ++partial) {
			fail_after = partials[partial];
			memcpy(bytes, original, sizeof(bytes));
			unsigned int before = transfers;
			CHECK(transfer(bytes, bytes + 1, NRF54L15_SPI_MAX_TRANSFER, 0) == errors[err]);
			CHECK(transfers == before + 1 && memcmp(bytes, original, sizeof(bytes)) == 0);
			CHECK(transfer(NULL, bytes, NRF54L15_SPI_MAX_TRANSFER, 0xff) == errors[err]);
			CHECK(memcmp(bytes, original, sizeof(bytes)) == 0);
			CHECK(transfer(bytes, NULL, 4, 0) == errors[err]);
		}
	}
	transfer_error = 0;
	CHECK(transfer(original, bytes, NRF54L15_SPI_MAX_TRANSFER, 0) == 0);
	for (size_t i = 0; i < NRF54L15_SPI_MAX_TRANSFER; ++i) {
		CHECK(bytes[i] == (uint8_t)(original[i] ^ 0x5aU));
	}
}

static void test_multiple_devices(void)
{
	reset_backend();
	init_default();
	static struct nrf54l15_spi other;
	struct spi_dt_spec spec = default_spec();
	spec.config.operation |= SPI_MODE_CPOL | SPI_MODE_CPHA;
	spec.config.frequency = 4000000;
	spec.config.slave = 1;
	spec.config.cs.gpio.pin = 5;
	CHECK(nrf54l15_spi_init(&other, &spec) == 0);
	for (unsigned int i = 0; i < 10; ++i) {
		CHECK(transfer(NULL, NULL, 1, 0) == 0);
		CHECK(last_spec == &spi.spec && captured_config.frequency == 1000000);
		CHECK(nrf54l15_spi_transfer(&other, NULL, NULL, 1, 0) == 0);
		CHECK(last_spec == &other.spec && captured_config.frequency == 4000000);
		CHECK(captured_config.slave == 1 && captured_config.cs.gpio.pin == 5);
	}
}

int main(void)
{
	test_validation();
	test_configurations();
	test_buffers();
	test_overlaps_and_partial_failures();
	test_multiple_devices();
	puts("spi tests passed");
	return 0;
}
