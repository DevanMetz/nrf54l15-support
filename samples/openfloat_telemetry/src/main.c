/*
 * OpenFloat telemetry on the Seeed XIAO nRF54L15 Sense.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <nrf54l15/button.h>
#include <nrf54l15/impact.h>
#include <nrf54l15/imu.h>
#include <nrf54l15/openfloat_ble.h>
#include <nrf54l15/orientation.h>
#include <nrf54l15/periodic.h>
#include <nrf54l15/telemetry.h>

#define SAMPLE_HZ 416
#define SAMPLE_PERIOD_US (1000000 / SAMPLE_HZ)
#define SERIAL_PRINT_DIVIDER 8
#define BLE_NOTIFY_DIVIDER 8
#define MADGWICK_BETA 0.08f
#define MOUNT_ROT_X_SIGN 1
#define SHOT_ACCEL_THRESHOLD_MPS2 117.72f
#define SHOT_RELEASE_THRESHOLD_MPS2 39.24f
#define SHOT_REFRACTORY_MS 800
#define LED_IDLE_PERIOD_US 500000
#define LED_SHOT_PULSE_US 120000
#define SCALE_MG (1000.0f / NRF54L15_GRAVITY_MPS2)
#define SCALE_GYRO_MDPS (NRF54L15_RAD_TO_DEG * 1000.0f)

static const struct device *const imu = DEVICE_DT_GET(DT_ALIAS(imu0));
static const struct gpio_dt_spec user_led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec user_btn = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});
static struct nrf54l15_imu imu_driver;
static struct nrf54l15_button button_driver;
static bool led_ready;
static atomic_t queue_drops;
static atomic_t missed_ticks;
static atomic_t sample_errors;

#define SEND_SERIAL BIT(0)
#define SEND_BLE BIT(1)
#define SHOT_EVENT BIT(2)
#define ZERO_EVENT BIT(3)

/* One snapshot owns all values used by the transport thread. No pointers into
 * live filter state cross the queue; enqueue is always nonblocking.
 */
struct telemetry_sample {
	uint64_t timestamp_us;
	uint32_t sequence;
	uint32_t dt_us;
	struct nrf54l15_vec3 accel;
	struct nrf54l15_vec3 gyro;
	struct nrf54l15_quat q;
	struct nrf54l15_vec3 angles;
	uint32_t shot_count;
	uint8_t events;
};

K_MSGQ_DEFINE(telemetry_queue, sizeof(struct telemetry_sample), 8, 4);

static void print_live(const struct telemetry_sample *s)
{
	printk("OFRAW,1,%u,%llu,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u\n",
	       s->sequence, (unsigned long long)s->timestamp_us, s->dt_us,
	       nrf54l15_scale_i32(s->accel.x, SCALE_MG),
	       nrf54l15_scale_i32(s->accel.y, SCALE_MG),
	       nrf54l15_scale_i32(s->accel.z, SCALE_MG),
	       nrf54l15_scale_i32(s->gyro.x, SCALE_GYRO_MDPS),
	       nrf54l15_scale_i32(s->gyro.y, SCALE_GYRO_MDPS),
	       nrf54l15_scale_i32(s->gyro.z, SCALE_GYRO_MDPS),
	       nrf54l15_scale_i32(s->angles.x, 100.0f),
	       nrf54l15_scale_i32(s->angles.y, 100.0f),
	       nrf54l15_scale_i32(s->angles.z, 100.0f),
	       nrf54l15_scale_i32(s->q.w, 1000000.0f),
	       nrf54l15_scale_i32(s->q.x, 1000000.0f),
	       nrf54l15_scale_i32(s->q.y, 1000000.0f),
	       nrf54l15_scale_i32(s->q.z, 1000000.0f), s->shot_count);
}

static void telemetry_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	struct telemetry_sample s;
	uint32_t ble_errors = 0;
	int64_t last_status_ms = 0;

	while (true) {
		int result = k_msgq_get(&telemetry_queue, &s, K_MSEC(1000));
		int64_t now_ms = k_uptime_get();
		if (now_ms - last_status_ms >= 1000) {
			last_status_ms = now_ms;
			printk("# OFSTATS,1,queue_drops=%u,missed_ticks=%u,sample_errors=%u,ble_errors=%u\n",
			       (uint32_t)atomic_get(&queue_drops), (uint32_t)atomic_get(&missed_ticks),
			       (uint32_t)atomic_get(&sample_errors), ble_errors);
		}
		if (result != 0) {
			continue;
		}
		if (s.events & SHOT_EVENT) {
			printk("OFSHOT,1,%u,%llu,%d,%d,%d,%u\n", s.shot_count,
			       (unsigned long long)s.timestamp_us,
			       nrf54l15_scale_i32(s.accel.x, SCALE_MG),
			       nrf54l15_scale_i32(s.accel.y, SCALE_MG),
			       nrf54l15_scale_i32(s.accel.z, SCALE_MG), s.shot_count);
		}
		if (s.events & ZERO_EVENT) {
			printk("# Calibrated: cant=0 pitch=0\n");
		}
		if (s.events & SEND_SERIAL) {
			print_live(&s);
		}
		if (s.events & SEND_BLE) {
			int err = nrf54l15_openfloat_ble_send(s.sequence, s.dt_us, &s.accel, &s.gyro);
			if (err && err != -ENOTCONN && err != -EACCES && err != -EAGAIN) {
				++ble_errors;
			}
		}
	}
}
K_THREAD_DEFINE(telemetry_worker, 2048, telemetry_thread, NULL, NULL, NULL, 7, 0, 0);

static struct nrf54l15_vec3 mounted(struct nrf54l15_vec3 raw)
{
	/* Preserve the existing +90 degree rotation around the forward/cant axis. */
	return (struct nrf54l15_vec3){raw.x, -MOUNT_ROT_X_SIGN * raw.z, MOUNT_ROT_X_SIGN * raw.y};
}

static void update_led(uint64_t now_us, uint64_t pulse_until_us)
{
	static bool previous;
	bool on = now_us < pulse_until_us || ((now_us / LED_IDLE_PERIOD_US) & 1U);

	if (led_ready && on != previous && gpio_pin_set_dt(&user_led, on) == 0) {
		previous = on;
	}
}

int main(void)
{
	struct nrf54l15_quat q = {1, 0, 0, 0};
	struct nrf54l15_impact impact;
	struct nrf54l15_periodic sample_clock;
	const struct nrf54l15_imu_config config = {SAMPLE_HZ, 16, 2000};
	uint64_t last_sample_us = 0, pulse_until_us = 0;
	uint32_t sequence = 0, shot_count = 0;
	float cant_offset = 0, pitch_offset = 0;
	int err;

	printk("# OPENFLOAT_PROTO,1\n# target: Seeed XIAO nRF54L15 Sense\n");
	printk("# sample_hz: %d\n# ui: user LED status, user button calibration\n", SAMPLE_HZ);
	printk("# ble: %s, notify every %d samples\n", CONFIG_BT_DEVICE_NAME, BLE_NOTIFY_DIVIDER);
	printk("# BLE live frame: 20 bytes, OF v1, seq u16, dt_us u16, accel_mg i16[3], gyro_dps_q4 i16[3]\n");
	printk("# format: OFSHOT,proto,shot_id,uptime_us,ax_mg,ay_mg,az_mg,shot_count\n");
	led_ready = user_led.port != NULL && gpio_is_ready_dt(&user_led) &&
		    gpio_pin_configure_dt(&user_led, GPIO_OUTPUT_INACTIVE) == 0;
	if (!led_ready) {
		printk("# User LED unavailable\n");
	}
	err = nrf54l15_button_init(&button_driver, &user_btn, 20);
	if (err) {
		printk("# User button configure failed: %d\n", err);
	}
	err = nrf54l15_impact_init(&impact, SHOT_ACCEL_THRESHOLD_MPS2,
				 SHOT_RELEASE_THRESHOLD_MPS2, SHOT_REFRACTORY_MS);
	if (err) {
		printk("# Impact detector configure failed: %d\n", err);
		return err;
	}
	/* Let the shared rail settle before the SDK driver's first bus access.
	 * The overlay defers initialization so a cold first boot is not latched
	 * permanently unavailable before main can wait for the sensor supply.
	 */
	k_sleep(K_MSEC(100));
	err = device_init(imu);
	if (err) {
		printk("# IMU device initialize failed: %d\n", err);
		return err;
	}
	err = nrf54l15_openfloat_ble_init();
	if (err) {
		printk("# BLE init failed: %d\n", err);
	}
	err = nrf54l15_imu_init(&imu_driver, imu, &config);
	if (err) {
		printk("# IMU configure failed: %d\n", err);
		return err;
	}
	printk("# IMU ready: %s, accel+gyro %d Hz, +/-16 g, +/-2000 dps\n", imu->name, SAMPLE_HZ);
	k_sleep(K_MSEC(100));
	err = nrf54l15_periodic_init(&sample_clock, CONFIG_SYS_CLOCK_TICKS_PER_SEC,
				   SAMPLE_HZ, (uint64_t)k_uptime_ticks());
	if (err) {
		printk("# Sample clock configure failed: %d\n", err);
		return err;
	}

	while (true) {
		uint64_t deadline, slots;
		err = nrf54l15_periodic_deadline(&sample_clock, &deadline);
		if (err || deadline > INT64_MAX) {
			return err ? err : -ERANGE;
		}
		k_sleep(K_TIMEOUT_ABS_TICKS((int64_t)deadline));
		err = nrf54l15_periodic_consume(&sample_clock, (uint64_t)k_uptime_ticks(), &slots);
		if (err || slots == 0) {
			continue;
		}
		atomic_add(&missed_ticks, (uint32_t)(slots - 1));
		sequence += (uint32_t)(slots - 1);
		uint32_t sample_sequence = sequence++;
		struct nrf54l15_imu_sample raw;
		err = nrf54l15_imu_read(&imu_driver, &raw);
		if (err) {
			atomic_inc(&sample_errors);
			continue;
		}
		uint64_t elapsed = last_sample_us == 0 ? SAMPLE_PERIOD_US : raw.timestamp_us - last_sample_us;
		last_sample_us = raw.timestamp_us;
		uint32_t dt_us = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
		float filter_dt = (dt_us == 0 || dt_us > 200000) ? 1.0f / SAMPLE_HZ : dt_us / 1000000.0f;
		struct nrf54l15_vec3 accel = mounted(raw.accel_mps2), gyro = mounted(raw.gyro_rps);
		struct nrf54l15_vec3 angles;
		err = nrf54l15_madgwick_update(&q, &gyro, &accel, MADGWICK_BETA, filter_dt);
		if (err || nrf54l15_quat_to_euler(&q, &angles) != 0) {
			atomic_inc(&sample_errors);
			continue;
		}
		uint8_t events = 0;
		uint32_t now_ms = (uint32_t)(raw.timestamp_us / 1000);
		if (nrf54l15_impact_update(&impact, &accel, now_ms) == 1) {
			++shot_count;
			pulse_until_us = raw.timestamp_us + LED_SHOT_PULSE_US;
			events |= SHOT_EVENT;
		}
		bool zero_requested = nrf54l15_openfloat_ble_take_zero_request();
		if (nrf54l15_button_poll(&button_driver, now_ms) == NRF54L15_BUTTON_PRESSED || zero_requested) {
			cant_offset = angles.x;
			pitch_offset = angles.y;
			events |= ZERO_EVENT;
		}
		angles.x = nrf54l15_angle_delta_deg(angles.x, cant_offset);
		angles.y = nrf54l15_angle_delta_deg(angles.y, pitch_offset);
		update_led(raw.timestamp_us, pulse_until_us);
		if (sample_sequence % SERIAL_PRINT_DIVIDER == 0) {
			events |= SEND_SERIAL;
		}
		if (sample_sequence % BLE_NOTIFY_DIVIDER == 0) {
			events |= SEND_BLE;
		}
		if (events) {
			struct telemetry_sample s = {
				.timestamp_us = raw.timestamp_us, .sequence = sample_sequence, .dt_us = dt_us,
				.accel = accel, .gyro = gyro, .q = q, .angles = angles, .shot_count = shot_count,
				.events = events,
			};
			if (k_msgq_put(&telemetry_queue, &s, K_NO_WAIT) != 0) {
				atomic_inc(&queue_drops);
			}
		}
	}
}
