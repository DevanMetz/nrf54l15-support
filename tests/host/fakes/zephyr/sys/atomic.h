#ifndef TEST_ZEPHYR_ATOMIC_H
#define TEST_ZEPHYR_ATOMIC_H
/* Sequential test double. These tests do not validate CPU memory ordering. */
typedef int atomic_t;
#define ATOMIC_INIT(value) (value)
static inline int atomic_get(const atomic_t *value) { return *value; }
static inline int atomic_set(atomic_t *value, int next)
{
	int old = *value;
	*value = next;
	return old;
}
static inline int atomic_clear(atomic_t *value) { return atomic_set(value, 0); }
#endif
