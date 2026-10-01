#ifndef TEST_ZEPHYR_I2C_H
#define TEST_ZEPHYR_I2C_H
#include <stdint.h>
#include <zephyr/device.h>
struct i2c_dt_spec { const struct device *bus; uint16_t addr; };
static inline bool i2c_is_ready_dt(const struct i2c_dt_spec *spec)
{
	return device_is_ready(spec->bus);
}
int i2c_reg_read_byte_dt(const struct i2c_dt_spec *, uint8_t, uint8_t *);
int i2c_reg_write_byte_dt(const struct i2c_dt_spec *, uint8_t, uint8_t);
int i2c_burst_read_dt(const struct i2c_dt_spec *, uint8_t, uint8_t *, uint32_t);
#endif
