/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <nrf54l15/imu.h>
#include <nrf54l15/button.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

static struct device device = {.ready = true};
static int attr_calls, fail_attr, fetch_error, channel_error, channel_calls;
static int gpio_error, gpio_value;
static uint32_t gpio_pin_seen, gpio_flags_seen;
static struct {
	enum sensor_channel channel;
	enum sensor_attribute attribute;
	float value;
} attrs[4];

int64_t k_uptime_ticks(void) { return 1234567; }

int sensor_attr_set(const struct device *dev, enum sensor_channel channel,
		    enum sensor_attribute attribute, const struct sensor_value *value)
{
	CHECK(dev == &device && attr_calls < 4);
	attrs[attr_calls].channel = channel;
	attrs[attr_calls].attribute = attribute;
	attrs[attr_calls].value = sensor_value_to_float(value);
	++attr_calls;
	return attr_calls == fail_attr ? -EIO : 0;
}

int sensor_sample_fetch(const struct device *dev)
{
	CHECK(dev == &device);
	return fetch_error;
}

int sensor_channel_get(const struct device *dev, enum sensor_channel channel,
		       struct sensor_value *values)
{
	CHECK(dev == &device);
	++channel_calls;
	values[0] = (struct sensor_value){.val1 = channel == SENSOR_CHAN_ACCEL_XYZ ? 9 : 1};
	values[1] = (struct sensor_value){.val1 = -2, .val2 = -500000};
	values[2] = (struct sensor_value){.val2 = 250000};
	/* Simulate a driver which writes some cached fields before failing. */
	return channel_calls == channel_error ? -EIO : 0;
}

int gpio_pin_configure_dt(const struct gpio_dt_spec *gpio, uint32_t flags)
{
	CHECK(flags == GPIO_INPUT);
	gpio_pin_seen = gpio->pin;
	gpio_flags_seen = gpio->dt_flags;
	return gpio_error;
}

int gpio_pin_get_dt(const struct gpio_dt_spec *gpio)
{
	CHECK(gpio->pin == gpio_pin_seen && gpio->dt_flags == gpio_flags_seen);
	return gpio_value;
}

static void imu_configuration(void)
{
	struct nrf54l15_imu imu = {0};
	struct nrf54l15_imu_config config = {416, 16, 2000};
	CHECK(nrf54l15_imu_init(&imu, &device, &config) == 0);
	CHECK(imu.ready && attr_calls == 4);
	CHECK(attrs[0].channel == SENSOR_CHAN_ACCEL_XYZ);
	CHECK(attrs[0].attribute == SENSOR_ATTR_FULL_SCALE);
	CHECK(fabsf(attrs[0].value - 156.9064f) < 0.001f);
	CHECK(attrs[1].channel == SENSOR_CHAN_GYRO_XYZ);
	CHECK(attrs[1].attribute == SENSOR_ATTR_FULL_SCALE);
	CHECK(fabsf(attrs[1].value - 34.90658f) < 0.001f);
	CHECK(attrs[2].channel == SENSOR_CHAN_ACCEL_XYZ && attrs[2].value == 416);
	CHECK(attrs[3].channel == SENSOR_CHAN_GYRO_XYZ && attrs[3].value == 416);
	for (fail_attr = 1; fail_attr <= 4; ++fail_attr) {
		attr_calls = 0;
		CHECK(nrf54l15_imu_init(&imu, &device, &config) == -EIO);
		CHECK(!imu.ready && attr_calls == fail_attr);
	}
	fail_attr = attr_calls = 0;
	config.accel_range_g = 12;
	CHECK(nrf54l15_imu_init(&imu, &device, &config) == -EINVAL);
	CHECK(!imu.ready && attr_calls == 0);
	config.accel_range_g = 16;
	device.ready = false;
	CHECK(nrf54l15_imu_init(&imu, &device, &config) == -ENODEV);
	CHECK(!imu.ready && attr_calls == 0);
	device.ready = true;
	CHECK(nrf54l15_imu_init(&imu, &device, &config) == 0);
	CHECK(imu.ready);
}

static void imu_read_failure(void)
{
	struct nrf54l15_imu imu = {.device = &device, .ready = true};
	struct nrf54l15_imu_sample output;
	memset(&output, 0x55, sizeof(output));
	unsigned char before[sizeof(output)];
	memcpy(before, &output, sizeof(output));
	fetch_error = -EIO;
	CHECK(nrf54l15_imu_read(&imu, &output) == -EIO);
	CHECK(channel_calls == 0 && memcmp(&output, before, sizeof(output)) == 0);
	fetch_error = 0;
	for (channel_error = 1; channel_error <= 2; ++channel_error) {
		channel_calls = 0;
		CHECK(nrf54l15_imu_read(&imu, &output) == -EIO);
		CHECK(memcmp(&output, before, sizeof(output)) == 0);
	}
	channel_error = channel_calls = 0;
	CHECK(nrf54l15_imu_read(&imu, &output) == 0);
	CHECK(output.timestamp_us == 1234567);
	CHECK(output.accel_mps2.x == 9 && output.accel_mps2.y == -2.5f);
	CHECK(output.gyro_rps.x == 1 && output.gyro_rps.z == 0.25f);
	imu.ready = false;
	CHECK(nrf54l15_imu_read(&imu, &output) == -ENODEV);
}

static void button_errors(void)
{
	struct gpio_dt_spec gpio = {.port = &device, .pin = 13, .dt_flags = 0x123};
	struct nrf54l15_button button;
	gpio_error = -EIO;
	CHECK(nrf54l15_button_init(&button, &gpio, 20) == -EIO);
	CHECK(nrf54l15_button_poll(&button, 0) == -ENODEV);
	gpio_error = 0;
	CHECK(nrf54l15_button_init(&button, &gpio, 20) == 0);
	CHECK(gpio_pin_seen == 13 && gpio_flags_seen == 0x123);
	gpio_value = 1;
	CHECK(nrf54l15_button_poll(&button, 0) == NRF54L15_BUTTON_NONE);
	gpio_value = -EIO;
	CHECK(nrf54l15_button_poll(&button, 18) == -EIO);
	gpio_value = 1;
	CHECK(nrf54l15_button_poll(&button, 20) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_button_poll(&button, 38) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_button_poll(&button, 40) == NRF54L15_BUTTON_PRESSED);
	CHECK(nrf54l15_button_poll(&button, 100) == NRF54L15_BUTTON_NONE);
	gpio_value = -EIO;
	CHECK(nrf54l15_button_poll(&button, 200) == -EIO);
	gpio_value = 1;
	CHECK(nrf54l15_button_poll(&button, 300) == NRF54L15_BUTTON_NONE);
	CHECK(nrf54l15_button_init(&button, &button.gpio, 20) == 0);
	CHECK(button.ready && button.gpio.pin == 13);
}

int main(void)
{
	imu_configuration();
	imu_read_failure();
	button_errors();
	puts("PASS: IMU range/rate configuration, device readiness, partial failures, GPIO errors");
	return 0;
}
