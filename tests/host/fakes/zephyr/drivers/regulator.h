/* Test doubles only. Firmware uses Zephyr's real regulator implementation. */
#ifndef TEST_ZEPHYR_REGULATOR_H
#define TEST_ZEPHYR_REGULATOR_H
#include <zephyr/device.h>
int regulator_enable(const struct device *dev);
int regulator_disable(const struct device *dev);
bool regulator_is_enabled(const struct device *dev);
#endif
