/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <nrf54l15/gpio_event.h>
#include <nrf54l15/lsm6ds3tr.h>
#include <nrf54l15/telemetry.h>

BUILD_ASSERT(!(DT_GPIO_FLAGS(DT_ALIAS(imu0), irq_gpios) & GPIO_ACTIVE_LOW),
	"Direct IMU driver uses active-high push-pull INT1");
static const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(DT_ALIAS(imu0));
static const struct gpio_dt_spec interrupt_pin = GPIO_DT_SPEC_GET(DT_ALIAS(imu0), irq_gpios);
static struct nrf54l15_lsm6ds3tr sensor;
static struct nrf54l15_gpio_event event;

int main(void)
{
	const struct nrf54l15_imu_config config = {416, 16, 2000};
	int err = nrf54l15_lsm6ds3tr_init(&sensor, &bus, &config);
	if (!err) { err = nrf54l15_lsm6ds3tr_fifo_start(&sensor); }
	if (!err) { err = nrf54l15_gpio_event_start(&event, &interrupt_pin); }
	if (!err) { err = nrf54l15_lsm6ds3tr_fifo_int1(&sensor, true); }
	if (!err) { printk("# IMU FIFO IRQ: 416 Hz, INT1 on P0.02, native sensor axes\n"); }
	uint32_t samples = 0, previous_irqs = 0;
	unsigned int max_batch = 0;
	int64_t last_data_ms = k_uptime_get(), last_report_ms = last_data_ms;
	struct nrf54l15_lsm6ds3tr_fifo_sample latest = {0};
	while (!err) {
		err = nrf54l15_gpio_event_wait(&event, 100);
		if (err) { break; }
		unsigned int batch = 0;
		for (; batch < 32; ++batch) {
			struct nrf54l15_lsm6ds3tr_fifo_sample next;
			err = nrf54l15_lsm6ds3tr_fifo_read(&sensor, &next);
			if (err == -EAGAIN) { err = 0; break; }
			if (err) { break; }
			latest = next;
			if (samples != UINT32_MAX) { ++samples; }
			last_data_ms = k_uptime_get();
		}
		if (err) { break; }
		if (batch > max_batch) { max_batch = batch; }
		int64_t now_ms = k_uptime_get();
		if (now_ms - last_data_ms >= 1000) { err = -ETIMEDOUT; break; }
		if (now_ms - last_report_ms >= 1000) {
			struct nrf54l15_gpio_event_stats stats;
			(void)nrf54l15_gpio_event_get_stats(&event, &stats);
			/* Level checks can wake without an edge, but actual callbacks must
			 * eventually arrive to establish that the interrupt path works. */
			if (stats.activations == previous_irqs && stats.activations != UINT32_MAX) {
				err = -ENXIO;
				break;
			}
			previous_irqs = stats.activations;
			printk("IMUIRQ,samples=%u,irqs=%u,max_batch=%u,fetch_us=%llu,accel_mg=%d/%d/%d\n",
				samples, stats.activations, max_batch,
				(unsigned long long)latest.sample.timestamp_us,
				nrf54l15_scale_i32(latest.sample.accel_mps2.x, 1000 / NRF54L15_GRAVITY_MPS2),
				nrf54l15_scale_i32(latest.sample.accel_mps2.y, 1000 / NRF54L15_GRAVITY_MPS2),
				nrf54l15_scale_i32(latest.sample.accel_mps2.z, 1000 / NRF54L15_GRAVITY_MPS2));
			last_report_ms = now_ms;
		}
	}
	int route_err = sensor.ready ? nrf54l15_lsm6ds3tr_fifo_int1(&sensor, false) : 0;
	int gpio_err = nrf54l15_gpio_event_stop(&event);
	int fifo_err = sensor.ready ? nrf54l15_lsm6ds3tr_fifo_stop(&sensor) : 0;
	printk("# IMU IRQ ended: error=%d route_stop=%d gpio_stop=%d fifo_stop=%d\n",
		err, route_err, gpio_err, fifo_err);
	/* Static event storage remains alive even when GPIO cleanup fails. */
	return 0;
}
