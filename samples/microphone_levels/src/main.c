/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <nrf54l15/dmic.h>
#include <nrf54l15/pcm16.h>

#define SAMPLE_RATE_HZ 16000
#define SAMPLES_PER_BLOCK 320
#define BLOCKS_PER_SESSION 500

K_MEM_SLAB_DEFINE_STATIC(audio_slab, SAMPLES_PER_BLOCK * sizeof(int16_t), 4, 4);
static struct nrf54l15_dmic mic;
static int16_t pcm[SAMPLES_PER_BLOCK];

int main(void)
{
	const struct nrf54l15_dmic_config config = {
		.sample_rate_hz = SAMPLE_RATE_HZ,
		.min_pdm_clock_hz = 1000000,
		.max_pdm_clock_hz = 3500000,
		.samples_per_block = SAMPLES_PER_BLOCK,
		.channel = PDM_CHAN_LEFT,
	};
	printk("# MIC: requested 16000 Hz mono PCM16; 20 ms blocks\n");
	printk("# MIC,session,block,peak,rms,mean,rail_samples (PCM code units)\n");
	k_sleep(K_MSEC(1000));
	for (uint32_t session = 0; ; ++session) {
		int err = nrf54l15_dmic_start(&mic, DEVICE_DT_GET(DT_ALIAS(dmic20)),
			&audio_slab, &config);
		/* Discard the first five blocks (nominally 100 ms) for settling. */
		for (uint32_t block = 0; !err && block < BLOCKS_PER_SESSION + 5; ++block) {
			err = nrf54l15_dmic_read(&mic, pcm, ARRAY_SIZE(pcm), 500);
			if (!err && block >= 5 && (block - 5) % 5 == 0) {
				struct nrf54l15_pcm16_levels levels;
				err = nrf54l15_pcm16_measure(pcm, ARRAY_SIZE(pcm), &levels);
				if (!err) {
					printk("MIC,%u,%u,%u,%u,%d,%u\n", session, block - 5,
						(unsigned int)levels.peak,
						(unsigned int)(levels.rms + 0.5f),
						(int)levels.mean, levels.rail_samples);
				}
			}
		}
		int stop_err = nrf54l15_dmic_stop(&mic, 1000);
		if (err || stop_err) {
			printk("# MIC failed: capture=%d cleanup=%d; reset before retry\n",
				err, stop_err);
			return 0; /* Static slab remains valid even if DMA cleanup failed. */
		}
		printk("# MIC session %u stopped; buffers returned; restarting in 1 s\n", session);
		k_sleep(K_MSEC(1000));
	}
}
