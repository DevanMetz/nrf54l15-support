/* SPDX-License-Identifier: Apache-2.0
 * Register definitions/scaling: ST LSM6DS3TR-C datasheet DocID030071 Rev 3,
 * tables 3/5 and sections 9.12-9.15, 9.27-9.40.
 * https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf
 * FIFO procedure/pattern: ST AN5130 sections 8.3-8.5.
 */
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <nrf54l15/lsm6ds3tr.h>

#define WHO_AM_I 0x0f
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_C 0x12
#define STATUS_REG 0x1e
#define OUT_TEMP_L 0x20
#define TEMPERATURE_DATA_READY 0x04
#define OUTX_L_G 0x22
#define EXPECTED_ID 0x6a
#define RESET_BIT 0x01
#define BDU_AUTO_INCREMENT 0x44
#define BOTH_DATA_READY 0x03
#define FIFO_CTRL1 0x06
#define FIFO_CTRL5 0x0a
#define FIFO_STATUS1 0x3a
#define FIFO_DATA_OUT_L 0x3e
#define FIFO_FULL_OR_OVERRUN 0x60
#define FIFO_EMPTY 0x10
#define INT1_CTRL 0x0d
#define INT1_FIFO_THRESHOLD 0x08

static int configuration(const struct nrf54l15_imu_config *config,
			 uint8_t control[3], float *accel_scale, float *gyro_scale)
{
	static const uint16_t rates[] = {26, 52, 104, 208, 416, 833};
	uint8_t odr = 0;
	for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i) {
		if (config->sample_hz == rates[i]) {
			odr = (uint8_t)((i + 2) << 4);
			break;
		}
	}
	if (odr == 0) {
		return -EINVAL;
	}
	uint8_t accel_fs, gyro_fs;
	float accel_mg, gyro_mdps;
	switch (config->accel_range_g) {
	case 2: accel_fs = 0x00; accel_mg = 0.061f; break;
	case 4: accel_fs = 0x08; accel_mg = 0.122f; break;
	case 8: accel_fs = 0x0c; accel_mg = 0.244f; break;
	case 16: accel_fs = 0x04; accel_mg = 0.488f; break;
	default: return -EINVAL;
	}
	switch (config->gyro_range_dps) {
	case 125: gyro_fs = 0x02; gyro_mdps = 4.375f; break;
	case 250: gyro_fs = 0x00; gyro_mdps = 8.75f; break;
	case 500: gyro_fs = 0x04; gyro_mdps = 17.5f; break;
	case 1000: gyro_fs = 0x08; gyro_mdps = 35.0f; break;
	case 2000: gyro_fs = 0x0c; gyro_mdps = 70.0f; break;
	default: return -EINVAL;
	}
	control[0] = odr | accel_fs;
	control[1] = odr | gyro_fs;
	control[2] = BDU_AUTO_INCREMENT;
	*accel_scale = accel_mg * (NRF54L15_GRAVITY_MPS2 / 1000.0f);
	*gyro_scale = gyro_mdps / (1000.0f * NRF54L15_RAD_TO_DEG);
	return 0;
}

int nrf54l15_lsm6ds3tr_init(struct nrf54l15_lsm6ds3tr *sensor,
	const struct i2c_dt_spec *bus, const struct nrf54l15_imu_config *config)
{
	if (sensor == NULL) {
		return -EINVAL;
	}
	sensor->ready = false;
	sensor->power = NRF54L15_LSM6DS3TR_POWER_FAULT;
	sensor->fifo_active = false;
	sensor->fifo_fault = false;
	sensor->discard_next = false;
	sensor->temperature_discard_next = true;
	sensor->fifo_sequence = 0;
	if (bus == NULL || bus->bus == NULL || config == NULL ||
	    (bus->addr != 0x6a && bus->addr != 0x6b)) {
		return -EINVAL;
	}
	uint8_t control[3];
	int err = configuration(config, control, &sensor->accel_scale, &sensor->gyro_scale);
	if (err) {
		return err;
	}
	sensor->run_control[0] = control[0];
	sensor->run_control[1] = control[1];
	/* FIFO ODR uses the same code at bit 3; continuous mode is 0b110. */
	sensor->fifo_control = (uint8_t)(((control[0] & 0xf0) >> 1) | 6);
	if (!i2c_is_ready_dt(bus)) {
		return -ENODEV;
	}
	sensor->bus = *bus;
	uint8_t value;
	err = i2c_reg_read_byte_dt(bus, WHO_AM_I, &value);
	if (err) {
		return err;
	}
	if (value != EXPECTED_ID) {
		return -ENODEV;
	}
	err = i2c_reg_write_byte_dt(bus, CTRL3_C, RESET_BIT);
	if (err) {
		return err;
	}
	for (int attempt = 0; attempt < 100; ++attempt) {
		k_sleep(K_MSEC(1));
		err = i2c_reg_read_byte_dt(bus, CTRL3_C, &value);
		if (err) {
			return err;
		}
		if (!(value & RESET_BIT)) {
			break;
		}
	}
	if (value & RESET_BIT) {
		return -ETIMEDOUT;
	}
	err = i2c_reg_write_byte_dt(bus, CTRL3_C, BDU_AUTO_INCREMENT);
	if (err) {
		return err;
	}
	err = i2c_reg_write_byte_dt(bus, CTRL1_XL, control[0]);
	if (err) {
		return err;
	}
	err = i2c_reg_write_byte_dt(bus, CTRL2_G, control[1]);
	if (err) {
		return err;
	}
	uint8_t readback[3];
	err = i2c_burst_read_dt(bus, CTRL1_XL, readback, sizeof(readback));
	if (err) {
		return err;
	}
	if (memcmp(control, readback, sizeof(control)) != 0) {
		return -EIO;
	}
	k_sleep(K_MSEC(100));
	sensor->ready = true;
	sensor->power = NRF54L15_LSM6DS3TR_RUNNING;
	return 0;
}

static int acquisition_error(const struct nrf54l15_lsm6ds3tr *sensor)
{
	if (!sensor->ready) { return -ENODEV; }
	if (sensor->power == NRF54L15_LSM6DS3TR_POWER_FAULT) { return -EPIPE; }
	if (sensor->power == NRF54L15_LSM6DS3TR_SUSPENDED) { return -EACCES; }
	return 0;
}

static int set_power(struct nrf54l15_lsm6ds3tr *sensor, bool running)
{
	if (!sensor) { return -EINVAL; }
	if (!sensor->ready) { return -ENODEV; }
	enum nrf54l15_lsm6ds3tr_power requested = running ?
		NRF54L15_LSM6DS3TR_RUNNING : NRF54L15_LSM6DS3TR_SUSPENDED;
	if (sensor->power == requested) { return 0; }
	/* No acquisition may trust a partially applied power/configuration change. */
	sensor->power = NRF54L15_LSM6DS3TR_POWER_FAULT;
	sensor->fifo_active = false;
	sensor->fifo_fault = true;
	sensor->fifo_sequence = 0;
	sensor->discard_next = true;
	sensor->temperature_discard_next = true;
	if (!i2c_is_ready_dt(&sensor->bus)) { return -ENODEV; }
	uint8_t value;
	int err = i2c_reg_read_byte_dt(&sensor->bus, WHO_AM_I, &value);
	if (err) { return err; }
	if (value != EXPECTED_ID) { return -ENODEV; }
	uint8_t control[3] = {
		running ? sensor->run_control[0] : sensor->run_control[0] & 0x0f,
		running ? sensor->run_control[1] : sensor->run_control[1] & 0x0f,
		BDU_AUTO_INCREMENT,
	};
	const uint8_t registers[] = {INT1_CTRL, FIFO_CTRL5, CTRL3_C, CTRL1_XL, CTRL2_G};
	const uint8_t values[] = {0, 0, control[2], control[0], control[1]};
	for (size_t i = 0; i < sizeof(registers); ++i) {
		err = i2c_reg_write_byte_dt(&sensor->bus, registers[i], values[i]);
		if (err) { return err; }
	}
	uint8_t readback[3];
	err = i2c_burst_read_dt(&sensor->bus, CTRL1_XL, readback, sizeof(readback));
	if (err) { return err; }
	if (memcmp(readback, control, sizeof(control))) { return -EIO; }
	err = i2c_reg_read_byte_dt(&sensor->bus, INT1_CTRL, &value);
	if (err) { return err; }
	if (value) { return -EIO; }
	err = i2c_reg_read_byte_dt(&sensor->bus, FIFO_CTRL5, &value);
	if (err) { return err; }
	if (value) { return -EIO; }
	if (running) { k_sleep(K_MSEC(100)); }
	sensor->fifo_fault = false;
	sensor->power = requested;
	return 0;
}

int nrf54l15_lsm6ds3tr_suspend(struct nrf54l15_lsm6ds3tr *sensor)
{
	return set_power(sensor, false);
}

int nrf54l15_lsm6ds3tr_resume(struct nrf54l15_lsm6ds3tr *sensor)
{
	return set_power(sensor, true);
}

static int16_t signed_le16(const uint8_t *data)
{
	uint16_t value = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
	return (int16_t)(value <= INT16_MAX ? (int32_t)value : (int32_t)value - 65536);
}

static struct nrf54l15_imu_sample convert_sample(const struct nrf54l15_lsm6ds3tr *sensor,
	const uint8_t data[12])
{
	return (struct nrf54l15_imu_sample){
		.timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks()),
		.gyro_rps = {signed_le16(&data[0]) * sensor->gyro_scale,
			     signed_le16(&data[2]) * sensor->gyro_scale,
			     signed_le16(&data[4]) * sensor->gyro_scale},
		.accel_mps2 = {signed_le16(&data[6]) * sensor->accel_scale,
			       signed_le16(&data[8]) * sensor->accel_scale,
			       signed_le16(&data[10]) * sensor->accel_scale},
	};
}

int nrf54l15_lsm6ds3tr_temperature_read(struct nrf54l15_lsm6ds3tr *sensor,
	struct nrf54l15_temperature_sample *sample)
{
	if (!sensor || !sample) { return -EINVAL; }
	int err = acquisition_error(sensor);
	if (err) { return err; }
	uint8_t status;
	err = i2c_reg_read_byte_dt(&sensor->bus, STATUS_REG, &status);
	if (err) { return err; }
	if (!(status & TEMPERATURE_DATA_READY)) { return -EAGAIN; }
	uint8_t data[2];
	err = i2c_burst_read_dt(&sensor->bus, OUT_TEMP_L, data, sizeof(data));
	if (err) { return err; }
	if (sensor->temperature_discard_next) {
		sensor->temperature_discard_next = false;
		return -EAGAIN;
	}
	/* Round the whole temperature, including the 25 C offset: rounding a
	 * negative delta first would round positive half-millidegrees downward.
	 */
	int32_t scaled = 25000 * 256 + (int32_t)signed_le16(data) * 1000;
	*sample = (struct nrf54l15_temperature_sample){
		.timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks()),
		.millicelsius = (scaled >= 0 ? scaled + 128 : scaled - 128) / 256,
	};
	return 0;
}

int nrf54l15_lsm6ds3tr_read(struct nrf54l15_lsm6ds3tr *sensor,
	struct nrf54l15_imu_sample *sample)
{
	if (sensor == NULL || sample == NULL) {
		return -EINVAL;
	}
	int err = acquisition_error(sensor);
	if (err) { return err; }
	if (sensor->fifo_active || sensor->fifo_fault) {
		return -EBUSY;
	}
	uint8_t status;
	err = i2c_reg_read_byte_dt(&sensor->bus, STATUS_REG, &status);
	if (err) {
		return err;
	}
	if ((status & BOTH_DATA_READY) != BOTH_DATA_READY) {
		return -EAGAIN;
	}
	uint8_t data[12];
	err = i2c_burst_read_dt(&sensor->bus, OUTX_L_G, data, sizeof(data));
	if (err) {
		return err;
	}
	if (sensor->discard_next) {
		sensor->discard_next = false;
		return -EAGAIN;
	}
	*sample = convert_sample(sensor, data);
	return 0;
}

int nrf54l15_lsm6ds3tr_fifo_start(struct nrf54l15_lsm6ds3tr *sensor)
{
	if (sensor == NULL) {
		return -EINVAL;
	}
	int err = acquisition_error(sensor);
	if (err) { return err; }
	sensor->fifo_fault = true;
	sensor->fifo_active = false;
	/* Bypass clears backlog before changing the FIFO configuration. */
	err = i2c_reg_write_byte_dt(&sensor->bus, FIFO_CTRL5, 0);
	if (err) {
		return err;
	}
	/* One complete pair per watermark; no changes to routing or extra data sets.
	 * Both sensors run at the same ODR, so the pattern is Gx,Gy,Gz,Ax,Ay,Az.
	 */
	uint8_t control[5] = {6, 0, 0x09, 0, sensor->fifo_control};
	for (uint8_t i = 0; i < sizeof(control); ++i) {
		err = i2c_reg_write_byte_dt(&sensor->bus, (uint8_t)(FIFO_CTRL1 + i), control[i]);
		if (err) {
			return err;
		}
	}
	uint8_t readback[5];
	err = i2c_burst_read_dt(&sensor->bus, FIFO_CTRL1, readback, sizeof(readback));
	if (err) {
		return err;
	}
	if (memcmp(readback, control, sizeof(control)) != 0) {
		return -EIO;
	}
	sensor->fifo_sequence = 0;
	sensor->discard_next = true;
	sensor->fifo_active = true;
	sensor->fifo_fault = false;
	return 0;
}

int nrf54l15_lsm6ds3tr_fifo_stop(struct nrf54l15_lsm6ds3tr *sensor)
{
	if (sensor == NULL) {
		return -EINVAL;
	}
	if (!sensor->ready) {
		return -ENODEV;
	}
	if (!sensor->fifo_active && !sensor->fifo_fault) {
		return 0;
	}
	sensor->fifo_fault = true;
	int err = i2c_reg_write_byte_dt(&sensor->bus, FIFO_CTRL5, 0);
	if (err) {
		return err;
	}
	uint8_t mode;
	err = i2c_reg_read_byte_dt(&sensor->bus, FIFO_CTRL5, &mode);
	if (err) {
		return err;
	}
	if (mode != 0) {
		return -EIO;
	}
	sensor->fifo_fault = false;
	sensor->fifo_active = false;
	sensor->discard_next = true;
	return 0;
}

int nrf54l15_lsm6ds3tr_fifo_int1(struct nrf54l15_lsm6ds3tr *sensor, bool enabled)
{
	if (!sensor) { return -EINVAL; }
	if (!sensor->ready) { return -ENODEV; }
	if (enabled) {
		int err = acquisition_error(sensor);
		if (err) { return err; }
	}
	if (enabled && sensor->fifo_fault) { return -EPIPE; }
	if (enabled && !sensor->fifo_active) { return -EACCES; }
	uint8_t route = enabled ? INT1_FIFO_THRESHOLD : 0;
	int err = i2c_reg_write_byte_dt(&sensor->bus, INT1_CTRL, route);
	if (err) { return err; }
	uint8_t readback;
	err = i2c_reg_read_byte_dt(&sensor->bus, INT1_CTRL, &readback);
	return err ? err : readback == route ? 0 : -EIO;
}

static int fifo_status(struct nrf54l15_lsm6ds3tr *sensor, uint16_t *words)
{
	uint8_t status[4];
	int err = i2c_burst_read_dt(&sensor->bus, FIFO_STATUS1, status, sizeof(status));
	if (err) {
		return err;
	}
	if (status[1] & FIFO_FULL_OR_OVERRUN) {
		/* Stop at full as well as overrun: continuity near capacity is uncertain. */
		return -EOVERFLOW;
	}
	/* With no unread word, the pattern register can retain the last popped
	 * word's index. It cannot establish alignment until data is available.
	 */
	if (status[1] & FIFO_EMPTY) {
		*words = 0;
		return 0;
	}
	uint16_t pattern = (uint16_t)(status[2] | ((uint16_t)(status[3] & 3) << 8));
	if (pattern != 0) {
		return -EILSEQ;
	}
	*words = (uint16_t)(status[0] | ((uint16_t)(status[1] & 7) << 8));
	return 0;
}

int nrf54l15_lsm6ds3tr_fifo_read(struct nrf54l15_lsm6ds3tr *sensor,
	struct nrf54l15_lsm6ds3tr_fifo_sample *sample)
{
	if (sensor == NULL || sample == NULL) {
		return -EINVAL;
	}
	int err = acquisition_error(sensor);
	if (err) { return err; }
	if (sensor->fifo_fault) {
		return -EPIPE;
	}
	if (!sensor->fifo_active) {
		return -EACCES;
	}
	uint16_t words;
	err = fifo_status(sensor, &words);
	if (!err && words < 6) {
		return -EAGAIN;
	}
	uint8_t data[12];
	/* Each read must address the two FIFO output registers again. A longer
	 * IF_INC burst continues beyond 0x3f into unrelated registers, consuming
	 * only one FIFO word and leaving the next pair misaligned.
	 */
	for (size_t offset = 0; !err && offset < sizeof(data); offset += 2) {
		err = i2c_burst_read_dt(&sensor->bus, FIFO_DATA_OUT_L, &data[offset], 2);
	}
	if (!err) {
		err = fifo_status(sensor, &words);
	}
	if (err) {
		sensor->fifo_fault = true;
		return err;
	}
	if (sensor->discard_next) {
		sensor->discard_next = false;
		return -EAGAIN;
	}
	*sample = (struct nrf54l15_lsm6ds3tr_fifo_sample){
		.sequence = sensor->fifo_sequence++, .sample = convert_sample(sensor, data),
	};
	return 0;
}
