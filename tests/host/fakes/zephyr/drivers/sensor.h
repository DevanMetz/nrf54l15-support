#ifndef TEST_ZEPHYR_SENSOR_H
#define TEST_ZEPHYR_SENSOR_H
#include <stdint.h>
#include <zephyr/device.h>
enum sensor_channel { SENSOR_CHAN_ACCEL_XYZ, SENSOR_CHAN_GYRO_XYZ, SENSOR_CHAN_DIE_TEMP };
enum sensor_attribute { SENSOR_ATTR_FULL_SCALE, SENSOR_ATTR_SAMPLING_FREQUENCY };
struct sensor_value { int32_t val1, val2; };
int sensor_attr_set(const struct device *, enum sensor_channel, enum sensor_attribute,
		    const struct sensor_value *);
int sensor_sample_fetch(const struct device *);
int sensor_sample_fetch_chan(const struct device *, enum sensor_channel);
int sensor_channel_get(const struct device *, enum sensor_channel, struct sensor_value *);
static inline float sensor_value_to_float(const struct sensor_value *value)
{
	return (float)value->val1 + (float)value->val2 / 1000000.0f;
}
static inline void sensor_g_to_ms2(int32_t g, struct sensor_value *value)
{
	int64_t micro = (int64_t)g * 9806650;
	value->val1 = (int32_t)(micro / 1000000);
	value->val2 = (int32_t)(micro % 1000000);
}
static inline void sensor_degrees_to_rad(int32_t d, struct sensor_value *value)
{
	int64_t micro = (int64_t)d * 3141592 / 180;
	value->val1 = (int32_t)(micro / 1000000);
	value->val2 = (int32_t)(micro % 1000000);
}
#endif
