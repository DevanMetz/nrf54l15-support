/* Test doubles only. Firmware uses the real Zephyr DMIC API. */
#ifndef TEST_ZEPHYR_AUDIO_DMIC_H
#define TEST_ZEPHYR_AUDIO_DMIC_H
#include <zephyr/device.h>
#include <zephyr/kernel.h>
enum pdm_lr {PDM_CHAN_LEFT, PDM_CHAN_RIGHT};
enum dmic_trigger {
	DMIC_TRIGGER_STOP, DMIC_TRIGGER_START, DMIC_TRIGGER_PAUSE,
	DMIC_TRIGGER_RELEASE, DMIC_TRIGGER_RESET,
};
struct pdm_io_cfg {
	uint32_t min_pdm_clk_freq, max_pdm_clk_freq;
	uint8_t min_pdm_clk_dc, max_pdm_clk_dc;
	uint8_t pdm_clk_pol, pdm_data_pol;
	uint32_t pdm_clk_skew;
};
struct pcm_stream_cfg {
	uint32_t pcm_rate;
	uint8_t pcm_width;
	uint16_t block_size;
	struct k_mem_slab *mem_slab;
};
struct pdm_chan_cfg {
	uint32_t req_chan_map_lo, req_chan_map_hi;
	uint32_t act_chan_map_lo, act_chan_map_hi;
	uint8_t req_num_chan, act_num_chan, req_num_streams, act_num_streams;
};
struct dmic_cfg {
	struct pdm_io_cfg io;
	struct pcm_stream_cfg *streams;
	struct pdm_chan_cfg channel;
};
static inline uint32_t dmic_build_channel_map(uint8_t channel, uint8_t pdm, enum pdm_lr lr)
{
	return ((uint32_t)(pdm * 2 + lr)) << (channel * 4);
}
int dmic_configure(const struct device *dev, struct dmic_cfg *config);
int dmic_trigger(const struct device *dev, enum dmic_trigger command);
int dmic_read(const struct device *dev, uint8_t stream, void **buffer,
	size_t *size, int32_t timeout);
#endif
