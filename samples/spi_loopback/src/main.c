/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include <zephyr/kernel.h>
#include <nrf54l15/spi.h>

static struct nrf54l15_spi ports[4];
static uint8_t tx[NRF54L15_SPI_MAX_TRANSFER];
static uint8_t rx[NRF54L15_SPI_MAX_TRANSFER];

int main(void)
{
	const uint32_t modes[] = {0, SPI_MODE_CPHA, SPI_MODE_CPOL, SPI_MODE_CPOL | SPI_MODE_CPHA};
	const size_t lengths[] = {1, 3, 16, 255, 256, NRF54L15_SPI_MAX_TRANSFER};
	for (size_t mode = 0; mode < ARRAY_SIZE(modes); ++mode) {
		struct spi_dt_spec spec = SPI_DT_SPEC_GET(DT_NODELABEL(loopback), SPI_WORD_SET(8));
		spec.config.operation |= modes[mode];
		int err = nrf54l15_spi_init(&ports[mode], &spec);
		if (err) {
			printk("# SPI mode %u init failed: %d\n", (unsigned int)mode, err);
			return 0;
		}
	}
	printk("# SPI wire loopback: D10 to D9, D8=SCK, D0=CS; requested 2 MHz\n");
	printk("# SPI,round,mode,bytes,PASS\n");
	k_sleep(K_MSEC(1000));
	for (uint32_t round = 0; ; ++round) {
		for (size_t mode = 0; mode < ARRAY_SIZE(modes); ++mode) {
			for (size_t n = 0; n < ARRAY_SIZE(lengths); ++n) {
				size_t length = lengths[n];
				for (size_t i = 0; i < length; ++i) {
					tx[i] = (uint8_t)((i * 73U + round) ^ (0xa5U + mode));
				}
				memset(rx, 0, length);
				int err = nrf54l15_spi_transfer(&ports[mode], tx, rx, length, 0xff);
				if (err) {
					printk("# SPI transfer failed: mode=%u bytes=%u error=%d\n",
						(unsigned int)mode, (unsigned int)length, err);
					return 0;
				}
				if (memcmp(tx, rx, length)) {
					size_t first = 0;
					while (tx[first] == rx[first]) { ++first; }
					printk("# SPI mismatch: mode=%u byte=%u expected=%02x received=%02x\n",
						(unsigned int)mode, (unsigned int)first, tx[first], rx[first]);
					return 0;
				}
				printk("SPI,%u,%u,%u,PASS\n", round, (unsigned int)mode, (unsigned int)length);
			}
		}
		k_sleep(K_MSEC(1000));
	}
}
