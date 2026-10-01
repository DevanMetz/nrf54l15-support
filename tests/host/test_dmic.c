/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/dmic.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

/* Model the SDK's two DMA buffers, completed queue, and delayed STOP IRQ.
 * Ownership checks catch double frees, leaked queued blocks, and premature
 * reconfiguration independently of the adapter's state transitions. */
static union { uint32_t alignment; char bytes[4 * 65532]; } storage;
static struct device dev = {.ready = true};
static struct k_mem_slab slab;
static enum {FREE, DMA, QUEUED, CALLER} owners[4];
static unsigned int queue[4], head, queued;
static unsigned int config_calls, disable_calls, start_calls, stop_calls, read_calls, frees;
static int config_error, disable_error, start_error, stop_error, read_error, bad_actual;
static bool configured, active, stopping, stuck, allocate_at_start, partial_start, late_block;
static int64_t now_ms, stopped_at;
static uint32_t release_delay;
static size_t returned_size;
static bool null_block;
static struct pcm_stream_cfg received_stream;
static struct pdm_io_cfg received_io;
static uint32_t received_map;

static const struct nrf54l15_dmic_config default_config = {
	.sample_rate_hz = 16000,
	.min_pdm_clock_hz = 1000000,
	.max_pdm_clock_hz = 3500000,
	.samples_per_block = 4,
	.channel = PDM_CHAN_LEFT,
};

static void reset_backend(void)
{
	slab = (struct k_mem_slab){.buffer = storage.bytes,
		.info = {.num_blocks = 4, .block_size = 8}};
	memset(owners, 0, sizeof(owners));
	head = queued = config_calls = disable_calls = start_calls = stop_calls = read_calls = frees = 0;
	config_error = disable_error = start_error = stop_error = read_error = bad_actual = 0;
	configured = active = stopping = stuck = partial_start = late_block = null_block = false;
	allocate_at_start = dev.ready = true;
	now_ms = stopped_at = 0;
	release_delay = 3;
	returned_size = SIZE_MAX;
}

static void *block_ptr(unsigned int index)
{
	return slab.buffer + index * slab.info.block_size;
}

static void allocate_dma(unsigned int index)
{
	CHECK(owners[index] == FREE);
	owners[index] = DMA;
	++slab.info.num_used;
	for (size_t i = 0; i < slab.info.block_size / 2; ++i) {
		int16_t value = (int16_t)(start_calls * 10 + index + i);
		memcpy((char *)block_ptr(index) + i * 2, &value, 2);
	}
}

static void publish(unsigned int index)
{
	CHECK(owners[index] == DMA && queued < 4);
	owners[index] = QUEUED;
	queue[(head + queued++) % 4] = index;
}

void k_mem_slab_free(struct k_mem_slab *pool, void *buffer)
{
	CHECK(pool == &slab && buffer != NULL);
	uintptr_t offset = (uintptr_t)buffer - (uintptr_t)slab.buffer;
	CHECK(offset % slab.info.block_size == 0);
	unsigned int index = (unsigned int)(offset / slab.info.block_size);
	CHECK(index < 4 && (owners[index] == DMA || owners[index] == CALLER));
	CHECK(slab.info.num_used > 0);
	owners[index] = FREE;
	--slab.info.num_used;
	++frees;
}

int64_t k_uptime_get(void) { return now_ms; }

void k_sleep(uint32_t ms)
{
	now_ms += ms;
	if (stopping && !stuck && now_ms >= stopped_at) {
		for (unsigned int i = 0; i < 4; ++i) {
			if (owners[i] == DMA) {
				if (late_block) {
					publish(i);
					late_block = false;
				} else {
					k_mem_slab_free(&slab, block_ptr(i));
				}
			}
		}
		active = stopping = false;
	}
}

int dmic_configure(const struct device *device, struct dmic_cfg *cfg)
{
	CHECK(device == &dev && cfg && cfg->streams);
	CHECK(cfg->channel.req_num_chan == 1 && cfg->channel.req_num_streams == 1);
	CHECK(cfg->channel.req_chan_map_hi == 0 && cfg->channel.req_chan_map_lo <= 1);
	if (!cfg->streams->pcm_rate) {
		++disable_calls;
		if (disable_error) { return disable_error; }
		if (active) { return -EBUSY; }
		CHECK(slab.info.num_used == 0 && queued == 0);
		configured = false;
		return 0;
	}
	++config_calls;
	CHECK(!active && slab.info.num_used == 0 && queued == 0);
	if (config_error) { return config_error; }
	CHECK(cfg->streams->pcm_width == 16 && cfg->streams->mem_slab == &slab);
	CHECK(cfg->streams->block_size == slab.info.block_size);
	CHECK(cfg->io.pdm_clk_pol == 0 && cfg->io.pdm_data_pol == 0 && cfg->io.pdm_clk_skew == 0);
	received_stream = *cfg->streams;
	received_io = cfg->io;
	received_map = cfg->channel.req_chan_map_lo;
	cfg->channel.act_num_chan = bad_actual == 1 ? 2 : 1;
	cfg->channel.act_num_streams = bad_actual == 2 ? 2 : 1;
	cfg->channel.act_chan_map_lo = received_map ^ (bad_actual == 3);
	cfg->channel.act_chan_map_hi = bad_actual == 4;
	configured = true;
	return 0;
}

int dmic_trigger(const struct device *device, enum dmic_trigger command)
{
	CHECK(device == &dev);
	if (command == DMIC_TRIGGER_START) {
		++start_calls;
		CHECK(configured && !active);
		if (!start_error || partial_start) {
			active = true;
			if (allocate_at_start) {
				allocate_dma(0);
				allocate_dma(1);
			}
		}
		return start_error;
	}
	CHECK(command == DMIC_TRIGGER_STOP);
	++stop_calls;
	if (stop_error) { return stop_error; }
	if (active && !stopping) {
		stopping = true;
		stopped_at = now_ms + release_delay;
	}
	return 0;
}

int dmic_read(const struct device *device, uint8_t stream, void **buffer,
	size_t *size, int32_t timeout)
{
	CHECK(device == &dev && stream == 0 && timeout >= 0 && configured);
	++read_calls;
	if (read_error) {
		*buffer = storage.bytes; /* No ownership transfer on a failed read. */
		return read_error;
	}
	if (null_block) { *buffer = NULL; return 0; }
	if (!queued) { return timeout == 0 ? -ENOMSG : -EAGAIN; }
	unsigned int index = queue[head];
	head = (head + 1) % 4;
	--queued;
	CHECK(owners[index] == QUEUED);
	owners[index] = CALLER;
	*buffer = block_ptr(index);
	*size = returned_size == SIZE_MAX ? slab.info.block_size : returned_size;
	return 0;
}

static void start_default(struct nrf54l15_dmic *mic)
{
	CHECK(nrf54l15_dmic_start(mic, &dev, &slab, &default_config) == 0);
	CHECK(mic->state == NRF54L15_DMIC_RUNNING);
}

static void test_validation(void)
{
	reset_backend();
	struct nrf54l15_dmic mic = {0};
	struct nrf54l15_dmic_config cfg = default_config;
	CHECK(nrf54l15_dmic_start(NULL, &dev, &slab, &cfg) == -EINVAL);
	CHECK(nrf54l15_dmic_start(&mic, NULL, &slab, &cfg) == -EINVAL);
	CHECK(nrf54l15_dmic_start(&mic, &dev, NULL, &cfg) == -EINVAL);
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, NULL) == -EINVAL);
	const uint32_t bad_rates[] = {0, 7999, 48001, UINT32_MAX};
	for (size_t i = 0; i < ARRAY_SIZE(bad_rates); ++i) {
		cfg.sample_rate_hz = bad_rates[i];
		CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	}
	cfg = default_config;
	const uint16_t bad_counts[] = {0, 1, 3, 32767, 32768, UINT16_MAX};
	for (size_t i = 0; i < ARRAY_SIZE(bad_counts); ++i) {
		cfg.samples_per_block = bad_counts[i];
		CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	}
	cfg = default_config;
	cfg.min_pdm_clock_hz = 0;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	cfg.min_pdm_clock_hz = cfg.max_pdm_clock_hz + 1;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	cfg = default_config;
	cfg.channel = (enum pdm_lr)2;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	cfg = default_config;
	slab.buffer = NULL;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	slab.buffer = storage.bytes + 1;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	slab.buffer = storage.bytes;
	slab.info.block_size = 12;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	slab.info.block_size = 8;
	slab.info.num_blocks = 3;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EINVAL);
	slab.info.num_blocks = 4;
	slab.info.num_used = 1;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EBUSY);
	slab.info.num_used = 0;
	dev.ready = false;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -ENODEV);
	CHECK(config_calls == 0 && start_calls == 0);
	int16_t out[4] = {0};
	CHECK(nrf54l15_dmic_read(&mic, out, 4, 0) == -ENODEV);
	CHECK(nrf54l15_dmic_read(NULL, out, 4, 0) == -EINVAL);
	CHECK(nrf54l15_dmic_stop(NULL, 0) == -EINVAL);
	CHECK(nrf54l15_dmic_stop(&mic, -1) == -EINVAL);
	CHECK(nrf54l15_dmic_stop(&mic, 0) == 0 && stop_calls == 0);
}

static void test_configurations(void)
{
	const uint32_t rates[] = {8000, 16000, 44100, 48000};
	const uint16_t counts[] = {2, 4, 320, 32766};
	for (unsigned int channel = 0; channel < 2; ++channel) {
		for (size_t rate = 0; rate < ARRAY_SIZE(rates); ++rate) {
			for (size_t count = 0; count < ARRAY_SIZE(counts); ++count) {
				reset_backend();
				struct nrf54l15_dmic mic = {0};
				struct nrf54l15_dmic_config cfg = default_config;
				cfg.sample_rate_hz = rates[rate];
				cfg.samples_per_block = counts[count];
				cfg.channel = (enum pdm_lr)channel;
				slab.info.block_size = counts[count] * 2U;
				CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == 0);
				CHECK(received_stream.pcm_rate == rates[rate]);
				CHECK(received_map == channel);
				CHECK(received_io.min_pdm_clk_freq == 1000000 && received_io.max_pdm_clk_freq == 3500000);
				CHECK(received_io.min_pdm_clk_dc == 40 && received_io.max_pdm_clk_dc == 60);
				CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &cfg) == -EBUSY);
				CHECK(config_calls == 1);
				CHECK(nrf54l15_dmic_stop(&mic, 20) == 0);
				CHECK(!configured && !active && slab.info.num_used == 0 && frees == 2);
			}
		}
	}
}

static void test_start_errors(void)
{
	reset_backend();
	struct nrf54l15_dmic mic = {0};
	config_error = -EINVAL;
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &default_config) == -EINVAL);
	CHECK(mic.state == NRF54L15_DMIC_IDLE && start_calls == 0);
	config_error = 0;
	start_default(&mic);
	CHECK(nrf54l15_dmic_stop(&mic, 20) == 0);
	for (int mismatch = 1; mismatch <= 4; ++mismatch) {
		reset_backend();
		bad_actual = mismatch;
		CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &default_config) == -ENOTSUP);
		CHECK(mic.state == NRF54L15_DMIC_STOPPING && start_calls == 0);
		CHECK(nrf54l15_dmic_stop(&mic, 0) == 0 && !configured);
	}
	for (int partial = 0; partial < 2; ++partial) {
		reset_backend();
		start_error = -EIO;
		partial_start = partial != 0;
		CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &default_config) == -EIO);
		CHECK(mic.state == NRF54L15_DMIC_STOPPING);
		CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &default_config) == -EBUSY);
		CHECK(nrf54l15_dmic_stop(&mic, 20) == 0 && slab.info.num_used == 0);
		start_error = 0;
		start_default(&mic);
		CHECK(nrf54l15_dmic_stop(&mic, 20) == 0);
	}
}

static void test_reads(void)
{
	reset_backend();
	struct nrf54l15_dmic mic = {0};
	start_default(&mic);
	int16_t out[6] = {-1, -1, -1, -1, -1, -1};
	const int16_t unchanged[6] = {-1, -1, -1, -1, -1, -1};
	CHECK(nrf54l15_dmic_read(&mic, NULL, 6, 0) == -EINVAL);
	CHECK(nrf54l15_dmic_read(&mic, out, 6, -1) == -EINVAL);
	CHECK(nrf54l15_dmic_read(&mic, out, 3, 0) == -EMSGSIZE && read_calls == 0);
	CHECK(nrf54l15_dmic_read(&mic, out, 6, 0) == -ENOMSG);
	CHECK(nrf54l15_dmic_read(&mic, out, 6, 500) == -EAGAIN);
	read_error = -EIO;
	CHECK(nrf54l15_dmic_read(&mic, out, 6, 500) == -EIO && frees == 0);
	read_error = 0;
	null_block = true;
	CHECK(nrf54l15_dmic_read(&mic, out, 6, 500) == -EIO && frees == 0);
	null_block = false;
	CHECK(memcmp(out, unchanged, sizeof(out)) == 0);
	publish(0);
	publish(1);
	CHECK(nrf54l15_dmic_read(&mic, out, 6, 500) == 0);
	CHECK(out[0] == 10 && out[1] == 11 && out[2] == 12 && out[3] == 13);
	CHECK(out[4] == -1 && out[5] == -1 && frees == 1);
	CHECK(nrf54l15_dmic_read(&mic, out, 6, 0) == 0 && out[0] == 11 && frees == 2);
	CHECK(nrf54l15_dmic_stop(&mic, 20) == 0);
	const size_t bad_sizes[] = {0, 6, 10, SIZE_MAX - 1};
	for (size_t i = 0; i < ARRAY_SIZE(bad_sizes); ++i) {
		reset_backend();
		start_default(&mic);
		publish(0);
		returned_size = bad_sizes[i];
		memcpy(out, unchanged, sizeof(out));
		CHECK(nrf54l15_dmic_read(&mic, out, 6, 500) == -EMSGSIZE);
		CHECK(frees == 1 && memcmp(out, unchanged, sizeof(out)) == 0);
		CHECK(nrf54l15_dmic_stop(&mic, 20) == 0);
	}
}

static void test_cleanup_and_restart(void)
{
	reset_backend();
	struct nrf54l15_dmic mic = {0};
	int16_t out[4];
	for (unsigned int session = 1; session <= 100; ++session) {
		start_default(&mic);
		publish(0);
		publish(1);
		CHECK(nrf54l15_dmic_read(&mic, out, 4, 100) == 0);
		CHECK(out[0] == (int16_t)(session * 10));
		CHECK(nrf54l15_dmic_stop(&mic, 20) == 0);
		CHECK(queued == 0 && slab.info.num_used == 0 && !configured && !active);
	}
	CHECK(frees == 200 && start_calls == 100);
	/* Empty slab is insufficient: clock/STOP completion is still pending. */
	reset_backend();
	allocate_at_start = false;
	start_default(&mic);
	CHECK(nrf54l15_dmic_stop(&mic, 0) == -ETIMEDOUT);
	CHECK(active && disable_calls == 1 && mic.state == NRF54L15_DMIC_STOPPING);
	CHECK(nrf54l15_dmic_read(&mic, out, 4, 0) == -EBUSY);
	CHECK(nrf54l15_dmic_start(&mic, &dev, &slab, &default_config) == -EBUSY);
	CHECK(nrf54l15_dmic_stop(&mic, 20) == 0 && now_ms == 3);
	/* A completion racing with STOP must also be drained. */
	reset_backend();
	late_block = true;
	start_default(&mic);
	CHECK(nrf54l15_dmic_stop(&mic, 20) == 0 && frees == 2 && queued == 0);
}

static void test_cleanup_errors(void)
{
	reset_backend();
	struct nrf54l15_dmic mic = {0};
	start_default(&mic);
	stuck = true;
	CHECK(nrf54l15_dmic_stop(&mic, 5) == -ETIMEDOUT && now_ms == 5);
	CHECK(slab.info.num_used == 2 && disable_calls == 0);
	CHECK(mic.state == NRF54L15_DMIC_STOPPING);
	stuck = false;
	CHECK(nrf54l15_dmic_stop(&mic, 20) == 0 && frees == 2);
	reset_backend();
	start_default(&mic);
	stop_error = -EIO;
	CHECK(nrf54l15_dmic_stop(&mic, 20) == -EIO && slab.info.num_used == 2);
	CHECK(mic.state == NRF54L15_DMIC_STOPPING);
	stop_error = 0;
	read_error = -EIO;
	CHECK(nrf54l15_dmic_stop(&mic, 20) == -EIO && frees == 0);
	read_error = 0;
	null_block = true;
	CHECK(nrf54l15_dmic_stop(&mic, 20) == -EIO);
	null_block = false;
	disable_error = -EIO;
	CHECK(nrf54l15_dmic_stop(&mic, 20) == -EIO && frees == 2);
	CHECK(mic.state == NRF54L15_DMIC_STOPPING && configured);
	disable_error = 0;
	CHECK(nrf54l15_dmic_stop(&mic, 0) == 0 && !configured);
	CHECK(nrf54l15_dmic_stop(&mic, 0) == 0);
}

int main(void)
{
	test_validation();
	test_configurations();
	test_start_errors();
	test_reads();
	test_cleanup_and_restart();
	test_cleanup_errors();
	puts("dmic tests passed");
	return 0;
}
