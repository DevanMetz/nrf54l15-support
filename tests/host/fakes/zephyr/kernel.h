#ifndef TEST_ZEPHYR_KERNEL_H
#define TEST_ZEPHYR_KERNEL_H
#include <stdint.h>
#include <stddef.h>
#define ARG_UNUSED(value) ((void)(value))
#define ARRAY_SIZE(value) (sizeof(value) / sizeof((value)[0]))
#define BUILD_ASSERT(condition, message) _Static_assert(condition, message)
int64_t k_uptime_ticks(void);
int64_t k_uptime_get(void);
#define K_MSEC(ms) (ms)
#define K_NO_WAIT 0
struct k_sem { unsigned int count; unsigned int limit; };
int k_sem_init(struct k_sem *, unsigned int, unsigned int);
void k_sem_give(struct k_sem *);
int k_sem_take(struct k_sem *, uint32_t);
void k_sem_reset(struct k_sem *);
void k_sleep(uint32_t ms);
static inline uint64_t k_ticks_to_us_floor64(int64_t ticks) { return (uint64_t)ticks; }
struct k_work { void (*handler)(struct k_work *); };
#define K_WORK_DEFINE(name, callback) struct k_work name = {callback}
int k_work_submit(struct k_work *work);
int printk(const char *format, ...);
struct k_mem_slab_info {
	uint32_t num_blocks;
	size_t block_size;
	uint32_t num_used;
};
struct k_mem_slab {
	char *buffer;
	struct k_mem_slab_info info;
};
static inline uint32_t k_mem_slab_num_used_get(struct k_mem_slab *slab)
{
	return slab->info.num_used;
}
void k_mem_slab_free(struct k_mem_slab *slab, void *buffer);
#endif
