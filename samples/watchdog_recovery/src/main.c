/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/hwinfo.h>
#include <nrf54l15/watchdog.h>

int main(void)
{
	uint32_t cause;
	int err = hwinfo_get_reset_cause(&cause);
	if (err) {
		printk("# Reset cause unavailable: %d; demonstration not armed\n", err);
		return err;
	}
	printk("WDRESET,0x%08x\n", cause);
	err = hwinfo_clear_reset_cause();
	if (err) {
		printk("# Cannot clear reset cause: %d; demonstration not armed\n", err);
		return err;
	}
	if (cause & RESET_WATCHDOG) {
		printk("# Watchdog reset observed; demonstration complete, watchdog not rearmed\n");
		return 0;
	}
	printk("# This example intentionally causes one watchdog reset\n");
	printk("# Feeding both slots, then withholding slot 1 while slot 0 continues\n");
	k_sleep(K_SECONDS(1));
	struct nrf54l15_watchdog watchdog = {0};
	err = nrf54l15_watchdog_start(&watchdog, DEVICE_DT_GET(DT_ALIAS(watchdog0)),
		2000, 2, WDT_OPT_PAUSE_HALTED_BY_DBG);
	if (err) {
		printk("# Watchdog startup failed: %d; reset before retrying\n", err);
		return err;
	}
	/* Two simulated workers share the loop for a deterministic demonstration.
	 * In an application, each worker feeds only after its own work completes.
	 */
	for (unsigned int round = 0; round < 8; ++round) {
		err = nrf54l15_watchdog_feed(&watchdog, 0);
		if (!err) {
			err = nrf54l15_watchdog_feed(&watchdog, 1);
		}
		if (err) {
			printk("# Watchdog feed failed: %d\n", err);
			return err;
		}
		printk("WDFEED,%u,both\n", round);
		k_sleep(K_MSEC(250));
	}
	printk("# Withholding slot 1; reset expected even while slot 0 is fed\n");
	while (true) {
		err = nrf54l15_watchdog_feed(&watchdog, 0);
		if (err) {
			printk("# Watchdog slot 0 feed failed: %d\n", err);
			return err;
		}
		k_sleep(K_MSEC(250));
	}
}
