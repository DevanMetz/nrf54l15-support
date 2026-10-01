/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nrf54l15/gpio_event.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static const struct gpio_driver_config port_config = {.port_pin_mask = UINT32_C(0x80000004)};
static struct device port = {.ready = true, .config = &port_config}, unrelated = {.ready = true};
static const struct gpio_dt_spec pin = {&port, 2, GPIO_ACTIVE_HIGH};
static struct nrf54l15_gpio_event event;
static struct gpio_callback *callbacks[4];
static unsigned int locks, configure_calls, disable_calls, enable_calls, add_calls, remove_calls, gives, takes;
static int configure_error, disable_error, enable_error, add_error, remove_error, get_error, sem_error;
static bool in_isr, pin_active, fire_on_enable, fire_on_get, fire_on_wait;
static uint32_t armed, last_timeout;

unsigned int irq_lock(void) { return locks++; }
void irq_unlock(unsigned int key) { CHECK(locks == key + 1); locks = key; }

static void fire(uint32_t pins)
{
	CHECK(!locks && !in_isr);
	in_isr = true;
	for (size_t i = 0; i < 4; ++i) {
		if (callbacks[i] && (pins & callbacks[i]->pin_mask)) {
			callbacks[i]->handler(&port, callbacks[i], pins);
		}
	}
	in_isr = false;
}

int k_sem_init(struct k_sem *sem, unsigned int count, unsigned int limit)
{
	CHECK(!locks && !in_isr && !count && limit == 1);
	*sem = (struct k_sem){count, limit};
	return sem_error;
}
void k_sem_give(struct k_sem *sem)
{
	CHECK(in_isr);
	++gives;
	if (sem->count < sem->limit) { ++sem->count; }
}
int k_sem_take(struct k_sem *sem, uint32_t timeout)
{
	CHECK(!locks && !in_isr);
	++takes;
	last_timeout = timeout;
	if (timeout && fire_on_wait) { fire_on_wait = false; fire(armed); }
	if (!sem->count) { return timeout ? -EAGAIN : -EBUSY; }
	--sem->count;
	return 0;
}
void k_sem_reset(struct k_sem *sem) { CHECK(!locks && !in_isr); sem->count = 0; }

int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, uint32_t flags)
{
	CHECK(!locks && !in_isr && spec->port == &port && flags == GPIO_INPUT);
	++configure_calls;
	if (!configure_error) { armed &= ~(UINT32_C(1) << spec->pin); }
	return configure_error;
}
int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *spec, uint32_t flags)
{
	CHECK(!locks && !in_isr && spec->port == &port);
	uint32_t mask = UINT32_C(1) << spec->pin;
	if (flags == GPIO_INT_DISABLE) {
		++disable_calls;
		if (!disable_error) { armed &= ~mask; }
		return disable_error;
	}
	CHECK(flags == GPIO_INT_EDGE_TO_ACTIVE);
	++enable_calls;
	armed |= mask; /* A failed operation may partially arm the pin. */
	if (fire_on_enable) { fire(mask); }
	return enable_error;
}
int gpio_add_callback(const struct device *dev, struct gpio_callback *cb)
{
	CHECK(locks && !in_isr && dev == &port);
	++add_calls;
	if (add_error) { return add_error; }
	for (size_t i = 0; i < 4; ++i) {
		if (!callbacks[i]) { callbacks[i] = cb; return 0; }
	}
	return -ENOMEM;
}
int gpio_remove_callback(const struct device *dev, struct gpio_callback *cb)
{
	CHECK(locks && !in_isr && dev == &port && !(armed & cb->pin_mask));
	++remove_calls;
	if (remove_error) { return remove_error; }
	for (size_t i = 0; i < 4; ++i) {
		if (callbacks[i] == cb) { callbacks[i] = NULL; return 0; }
	}
	return -EINVAL;
}
int gpio_pin_get_dt(const struct gpio_dt_spec *spec)
{
	CHECK(!locks && !in_isr && spec->port == &port);
	if (fire_on_get) { fire_on_get = false; fire(UINT32_C(1) << spec->pin); }
	return get_error ? get_error : pin_active;
}

static void reset(void)
{
	CHECK(!event.bound);
	event = (struct nrf54l15_gpio_event){0};
	memset(callbacks, 0, sizeof(callbacks));
	locks = configure_calls = disable_calls = enable_calls = add_calls = remove_calls = gives = takes = 0;
	configure_error = disable_error = enable_error = add_error = remove_error = get_error = sem_error = 0;
	in_isr = pin_active = fire_on_enable = fire_on_get = fire_on_wait = false;
	armed = last_timeout = 0;
	port.ready = true;
	port.config = &port_config;
}

static struct nrf54l15_gpio_event_stats stats(void)
{
	struct nrf54l15_gpio_event_stats value;
	CHECK(nrf54l15_gpio_event_get_stats(&event, &value) == 0 && !locks);
	return value;
}

static void test_validation(void)
{
	reset();
	CHECK(nrf54l15_gpio_event_start(NULL, &pin) == -EINVAL);
	CHECK(nrf54l15_gpio_event_start(&event, NULL) == -EINVAL);
	struct gpio_dt_spec bad = pin;
	bad.port = NULL;
	CHECK(nrf54l15_gpio_event_start(&event, &bad) == -EINVAL);
	bad = pin; bad.pin = 32;
	CHECK(nrf54l15_gpio_event_start(&event, &bad) == -EINVAL);
	bad = pin; bad.dt_flags = GPIO_PULL_UP | GPIO_PULL_DOWN;
	CHECK(nrf54l15_gpio_event_start(&event, &bad) == -EINVAL);
	bad.dt_flags = 0x8000;
	CHECK(nrf54l15_gpio_event_start(&event, &bad) == -EINVAL);
	port.ready = false;
	CHECK(nrf54l15_gpio_event_start(&event, &pin) == -ENODEV);
	CHECK(!configure_calls && !add_calls);
	port.ready = true;
	bad = pin; bad.pin = 3;
	CHECK(nrf54l15_gpio_event_start(&event, &bad) == -EINVAL);
	port.config = NULL;
	CHECK(nrf54l15_gpio_event_start(&event, &pin) == -EINVAL);
	CHECK(!configure_calls && !add_calls);
	CHECK(nrf54l15_gpio_event_wait(NULL, 1) == -EINVAL);
	CHECK(nrf54l15_gpio_event_wait(&event, UINT32_MAX) == -EINVAL);
	CHECK(nrf54l15_gpio_event_wait(&event, 1) == -ENODEV);
	CHECK(nrf54l15_gpio_event_stop(NULL) == -EINVAL);
	CHECK(nrf54l15_gpio_event_stop(&event) == 0);
	CHECK(nrf54l15_gpio_event_get_stats(NULL, &event.stats) == -EINVAL);
	CHECK(nrf54l15_gpio_event_get_stats(&event, NULL) == -EINVAL);
	CHECK(!stats().activations && !stats().running);
}

static void test_wait_and_coalescing(void)
{
	reset();
	CHECK(nrf54l15_gpio_event_start(&event, &pin) == 0);
	CHECK(nrf54l15_gpio_event_start(&event, &pin) == -EALREADY);
	CHECK(nrf54l15_gpio_event_wait(&event, 0) == -EBUSY);
	CHECK(nrf54l15_gpio_event_wait(&event, 250) == -EAGAIN && last_timeout == 250);
	pin_active = true;
	CHECK(nrf54l15_gpio_event_wait(&event, 250) == 0 && last_timeout == 0);
	CHECK(!stats().activations); /* Level is not an observed activation edge. */
	pin_active = false;
	for (int i = 0; i < 1000; ++i) { fire(4); }
	CHECK(event.pending.count == 1 && stats().activations == 1000);
	CHECK(nrf54l15_gpio_event_wait(&event, 0) == 0);
	CHECK(nrf54l15_gpio_event_wait(&event, 0) == -EBUSY);
	fire_on_get = true; /* Callback arrives after empty poll but before wait. */
	CHECK(nrf54l15_gpio_event_wait(&event, 100) == 0);
	fire_on_wait = true;
	CHECK(nrf54l15_gpio_event_wait(&event, 100) == 0);
	get_error = -EIO;
	CHECK(nrf54l15_gpio_event_wait(&event, 1) == -EIO);
	CHECK(stats().activations == 1002 && stats().running);
	in_isr = true;
	event.callback.handler(&unrelated, &event.callback, 4);
	event.callback.handler(&port, &event.callback, 8);
	in_isr = false;
	CHECK(stats().activations == 1002);
	event.stats.activations = UINT32_MAX - 1;
	fire(4); fire(4);
	CHECK(stats().activations == UINT32_MAX);
	CHECK(nrf54l15_gpio_event_stop(&event) == 0 && !event.bound && !armed);
	CHECK(!stats().running && stats().activations == UINT32_MAX && event.pending.count == 0);
	CHECK(configure_calls == 2 && remove_calls == 1);
}

static void test_start_failures(void)
{
	for (int stage = 0; stage < 5; ++stage) {
		reset();
		int *failure[] = {&sem_error, &configure_error, &disable_error, &add_error, &enable_error};
		*failure[stage] = -EIO;
		fire_on_enable = true;
		CHECK(nrf54l15_gpio_event_start(&event, &pin) == -EIO);
		CHECK(event.bound == (stage == 4) && !stats().running);
		if (event.bound) {
			CHECK(nrf54l15_gpio_event_wait(&event, 0) == -EPIPE);
			CHECK(stats().activations == 1);
			fire(4);
			CHECK(stats().activations == 1); /* Late callback is harmless. */
		}
		*failure[stage] = 0;
		CHECK(nrf54l15_gpio_event_stop(&event) == 0 && !event.bound);
		CHECK(nrf54l15_gpio_event_start(&event, &pin) == 0);
		CHECK(stats().running && stats().activations == 1);
		CHECK(nrf54l15_gpio_event_stop(&event) == 0);
	}
}

static void test_cleanup_and_shared_port(void)
{
	for (int stage = 0; stage < 3; ++stage) {
		reset();
		CHECK(nrf54l15_gpio_event_start(&event, &pin) == 0);
		int *failure[] = {&disable_error, &configure_error, &remove_error};
		*failure[stage] = -EIO;
		CHECK(nrf54l15_gpio_event_stop(&event) == -EIO && event.bound);
		CHECK(!stats().running && nrf54l15_gpio_event_wait(&event, 0) == -EPIPE);
		CHECK(nrf54l15_gpio_event_start(&event, &pin) == -EALREADY);
		fire(4);
		CHECK(!stats().activations);
		*failure[stage] = 0;
		CHECK(nrf54l15_gpio_event_stop(&event) == 0 && !event.bound);
		CHECK(nrf54l15_gpio_event_stop(&event) == 0);
	}
	reset();
	struct nrf54l15_gpio_event second = {0};
	struct gpio_dt_spec other_pin = {&port, 31, GPIO_ACTIVE_LOW | GPIO_PULL_UP};
	CHECK(nrf54l15_gpio_event_start(&event, &pin) == 0);
	CHECK(nrf54l15_gpio_event_start(&second, &other_pin) == 0);
	for (int i = 0; i < 100; ++i) {
		fire(UINT32_C(0x80000004));
		CHECK(nrf54l15_gpio_event_wait(&event, 0) == 0);
		CHECK(nrf54l15_gpio_event_wait(&second, 0) == 0);
		CHECK(nrf54l15_gpio_event_stop(&event) == 0);
		CHECK(armed == UINT32_C(0x80000000));
		CHECK(nrf54l15_gpio_event_start(&event, &event.pin) == 0);
	}
	CHECK(second.stats.activations == 100 && event.pin.pin == 2);
	CHECK(nrf54l15_gpio_event_stop(&event) == 0);
	CHECK(nrf54l15_gpio_event_stop(&second) == 0 && !armed);
}

int main(void)
{
	test_validation();
	test_wait_and_coalescing();
	test_start_failures();
	test_cleanup_and_shared_port();
	puts("GPIO event tests passed");
	return 0;
}
