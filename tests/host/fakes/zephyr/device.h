/* Test doubles only. Firmware builds always use the real Zephyr headers. */
#ifndef TEST_ZEPHYR_DEVICE_H
#define TEST_ZEPHYR_DEVICE_H
#include <stdbool.h>
#include <stddef.h>
struct device { bool ready; const void *config; };
static inline bool device_is_ready(const struct device *dev) { return dev->ready; }
#endif
