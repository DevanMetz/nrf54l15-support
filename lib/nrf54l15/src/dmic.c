/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include <nrf54l15/dmic.h>

int nrf54l15_dmic_start(struct nrf54l15_dmic *mic, const struct device *dev,
	struct k_mem_slab *slab, const struct nrf54l15_dmic_config *config)
{
	if (!mic || !dev || !slab || !config) {
		return -EINVAL;
	}
	if (mic->state != NRF54L15_DMIC_IDLE) {
		return -EBUSY;
	}
	if (config->sample_rate_hz < 8000 || config->sample_rate_hz > 48000 ||
	    !config->min_pdm_clock_hz || config->min_pdm_clock_hz > config->max_pdm_clock_hz ||
	    config->samples_per_block < 2 || config->samples_per_block > 32766 ||
	    config->samples_per_block % 2 ||
	    (config->channel != PDM_CHAN_LEFT && config->channel != PDM_CHAN_RIGHT)) {
		return -EINVAL;
	}
	uint16_t bytes = (uint16_t)(config->samples_per_block * sizeof(int16_t));
	/* NCS v3.3.0 exposes slab geometry in info; no block-size getter exists. */
	if (!slab->buffer || (uintptr_t)slab->buffer % 4 ||
	    slab->info.block_size != bytes || slab->info.num_blocks < 4) {
		return -EINVAL;
	}
	if (k_mem_slab_num_used_get(slab) != 0) {
		return -EBUSY;
	}
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	struct pcm_stream_cfg stream = {
		.pcm_rate = config->sample_rate_hz,
		.pcm_width = 16,
		.block_size = bytes,
		.mem_slab = slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			.min_pdm_clk_freq = config->min_pdm_clock_hz,
			.max_pdm_clk_freq = config->max_pdm_clock_hz,
			.min_pdm_clk_dc = 40,
			.max_pdm_clk_dc = 60,
		},
		.streams = &stream,
		.channel = {
			.req_num_streams = 1,
			.req_num_chan = 1,
			.req_chan_map_lo = dmic_build_channel_map(0, 0, config->channel),
		},
	};
	int err = dmic_configure(dev, &cfg);
	if (err) {
		return err;
	}
	mic->dev = dev;
	mic->slab = slab;
	mic->block_bytes = bytes;
	mic->state = NRF54L15_DMIC_STOPPING;
	if (cfg.channel.act_num_streams != 1 || cfg.channel.act_num_chan != 1 ||
	    cfg.channel.act_chan_map_lo != cfg.channel.req_chan_map_lo ||
	    cfg.channel.act_chan_map_hi != 0) {
		return -ENOTSUP;
	}
	err = dmic_trigger(dev, DMIC_TRIGGER_START);
	if (!err) {
		mic->state = NRF54L15_DMIC_RUNNING;
	}
	return err;
}

int nrf54l15_dmic_read(struct nrf54l15_dmic *mic, int16_t *samples,
	size_t capacity, int32_t timeout_ms)
{
	if (!mic || !samples || timeout_ms < 0) {
		return -EINVAL;
	}
	if (mic->state != NRF54L15_DMIC_RUNNING) {
		return mic->state == NRF54L15_DMIC_IDLE ? -ENODEV : -EBUSY;
	}
	if (capacity < mic->block_bytes / sizeof(int16_t)) {
		return -EMSGSIZE;
	}
	void *buffer = NULL;
	size_t bytes = 0;
	int err = dmic_read(mic->dev, 0, &buffer, &bytes, timeout_ms);
	if (err) {
		return err;
	}
	if (!buffer) {
		return -EIO;
	}
	if (bytes == mic->block_bytes) {
		memcpy(samples, buffer, bytes);
	} else {
		err = -EMSGSIZE;
	}
	k_mem_slab_free(mic->slab, buffer);
	return err;
}

int nrf54l15_dmic_stop(struct nrf54l15_dmic *mic, int32_t timeout_ms)
{
	if (!mic || timeout_ms < 0) {
		return -EINVAL;
	}
	if (mic->state == NRF54L15_DMIC_IDLE) {
		return 0;
	}
	mic->state = NRF54L15_DMIC_STOPPING;
	int64_t began_ms = k_uptime_get();
	int err = dmic_trigger(mic->dev, DMIC_TRIGGER_STOP);
	if (err) {
		return err;
	}
	for (;;) {
		/* STOP is asynchronous and does not empty nrfx's completed-block queue.
		 * Bound each pass even if a faulty backend continues producing data. */
		for (uint32_t i = 0; i < mic->slab->info.num_blocks &&
		     k_mem_slab_num_used_get(mic->slab); ++i) {
			void *buffer = NULL;
			size_t bytes = 0;
			err = dmic_read(mic->dev, 0, &buffer, &bytes, 0);
			if (err == -EAGAIN || err == -ENOMSG) {
				break;
			}
			if (err || !buffer) {
				return err ? err : -EIO;
			}
			k_mem_slab_free(mic->slab, buffer);
		}
		if (k_mem_slab_num_used_get(mic->slab) == 0) {
			/* All buffers can be free before START/STOP interrupts finish.
			 * Zero rate disables the stream; nrfx returns EBUSY while active.
			 * This is the idle barrier, not merely a slab-count heuristic. */
			struct pcm_stream_cfg stream = {0};
			struct dmic_cfg cfg = {
				.streams = &stream,
				.channel = {.req_num_chan = 1, .req_num_streams = 1},
			};
			err = dmic_configure(mic->dev, &cfg);
			if (!err) {
				*mic = (struct nrf54l15_dmic){0};
				return 0;
			}
			if (err != -EBUSY) {
				return err;
			}
		}
		if (k_uptime_get() - began_ms >= timeout_ms) {
			return -ETIMEDOUT;
		}
		k_sleep(K_MSEC(1));
	}
}
