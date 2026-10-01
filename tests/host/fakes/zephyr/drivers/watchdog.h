#ifndef TEST_ZEPHYR_WATCHDOG_H
#define TEST_ZEPHYR_WATCHDOG_H
#include <stdint.h>
#include <zephyr/device.h>
#define WDT_OPT_PAUSE_IN_SLEEP 1
#define WDT_OPT_PAUSE_HALTED_BY_DBG 2
#define WDT_FLAG_RESET_SOC 2
struct wdt_window { uint32_t min, max; };
typedef void (*wdt_callback_t)(const struct device *, int);
struct wdt_timeout_cfg {
	struct wdt_window window;
	wdt_callback_t callback;
	uint8_t flags;
};
int wdt_install_timeout(const struct device *, const struct wdt_timeout_cfg *);
int wdt_setup(const struct device *, uint8_t);
int wdt_feed(const struct device *, int);
int wdt_disable(const struct device *);
#endif
