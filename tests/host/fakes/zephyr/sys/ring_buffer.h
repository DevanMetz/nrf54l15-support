/* Sequential queue double; tests adapter ownership, not Zephyr's ring algorithm. */
#ifndef TEST_ZEPHYR_RING_BUFFER_H
#define TEST_ZEPHYR_RING_BUFFER_H
#include <stdint.h>
struct ring_buf { uint8_t *buffer; uint32_t size, head, count; };
void test_ring_guard(int producer);
static inline void ring_buf_init(struct ring_buf *ring, uint32_t size, uint8_t *data)
{
	*ring = (struct ring_buf){.buffer = data, .size = size};
}
static inline uint32_t ring_buf_put(struct ring_buf *ring, const uint8_t *data, uint32_t count)
{
	test_ring_guard(1);
	uint32_t copied = count < ring->size - ring->count ? count : ring->size - ring->count;
	for (uint32_t i = 0; i < copied; ++i) {
		ring->buffer[(ring->head + ring->count + i) % ring->size] = data[i];
	}
	ring->count += copied;
	return copied;
}
static inline uint32_t ring_buf_get(struct ring_buf *ring, uint8_t *out, uint32_t capacity)
{
	test_ring_guard(0);
	uint32_t copied = capacity < ring->count ? capacity : ring->count;
	for (uint32_t i = 0; i < copied; ++i) { out[i] = ring->buffer[(ring->head + i) % ring->size]; }
	ring->head = (ring->head + copied) % ring->size;
	ring->count -= copied;
	return copied;
}
static inline uint32_t ring_buf_size_get(const struct ring_buf *ring)
{
	test_ring_guard(0);
	return ring->count;
}
static inline void ring_buf_reset(struct ring_buf *ring)
{
	test_ring_guard(0);
	ring->head = ring->count = 0;
}
#endif
