/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <nrf54l15/pwm.h>

static const struct pwm_dt_spec pin = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));

int main(void)
{
	struct nrf54l15_pwm output;
	int err = nrf54l15_pwm_init(&output, &pin);
	if (err) {
		printk("# PWM init failed: %d\n", err);
		return err;
	}
	printk("# PWM on XIAO D0/A0 (P1.04), period_ns=%u\n", pin.period);
	printk("# PWM,duty_permille (external LED or scope)\n");
	int duty = 0;
	int step = 10;
	while (true) {
		err = nrf54l15_pwm_set_duty_permille(&output, (uint16_t)duty);
		if (err) {
			printk("# PWM update failed: %d\n", err);
			int inactive_err = nrf54l15_pwm_set_duty_permille(&output, 0);
			if (inactive_err) {
				printk("# PWM inactive request also failed: %d\n", inactive_err);
			}
			return err;
		}
		if (duty % 250 == 0) {
			printk("PWM,%d\n", duty);
		}
		if (duty == 1000) {
			step = -10;
		} else if (duty == 0) {
			step = 10;
		}
		duty += step;
		k_sleep(K_MSEC(10));
	}
}
