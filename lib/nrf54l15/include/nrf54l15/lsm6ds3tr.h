/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NRF54L15_SUPPORT_LSM6DS3TR_H
#define NRF54L15_SUPPORT_LSM6DS3TR_H
#include <zephyr/drivers/i2c.h>
#include <nrf54l15/imu.h>
#include <nrf54l15/temperature.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Direct I2C polling driver for the LSM6DS3TR-C (WHO_AM_I=0x6a).
 * Exclusive ownership of this sensor is required. Disable its Zephyr sensor
 * node; the I2C controller and IMU power supply must remain enabled.
 * Optional FIFO streaming uses equal accel/gyro rates, no decimation, and
 * checks the six-word pattern. Optional FIFO watermark on INT1 and explicit
 * register-controlled suspend/resume. No SPI or automatic Zephyr PM hooks.
 * GPIO wake notification is provided by gpio_event.h.
 */
enum nrf54l15_lsm6ds3tr_power {
	NRF54L15_LSM6DS3TR_RUNNING,
	NRF54L15_LSM6DS3TR_SUSPENDED,
	NRF54L15_LSM6DS3TR_POWER_FAULT,
};

struct nrf54l15_lsm6ds3tr {
	struct i2c_dt_spec bus;
	float accel_scale;
	float gyro_scale;
	bool ready; /* Init succeeded; acquisition also requires power == RUNNING. */
	bool fifo_active;
	bool fifo_fault;
	bool discard_next;
	bool temperature_discard_next;
	uint8_t fifo_control;
	uint32_t fifo_sequence;
	uint8_t run_control[2]; /* CTRL1_XL / CTRL2_G from successful init. */
	enum nrf54l15_lsm6ds3tr_power power;
};

/* Resets the sensor, enables block-data-update/autoincrement, configures and
 * verifies ranges/rates, then waits 100 ms to settle. Thread context only.
 * Rates: 26, 52, 104, 208, 416, 833 Hz. Ranges match nrf54l15_imu_config.
 * All I2C errors are propagated; failed initialization leaves ready=false.
 */
int nrf54l15_lsm6ds3tr_init(struct nrf54l15_lsm6ds3tr *sensor,
	const struct i2c_dt_spec *bus, const struct nrf54l15_imu_config *config);

/* Suspend clears INT1 routing and FIFO, then powers down accel and gyro by
 * setting both ODR fields to zero. Resume restores init's ranges/rate/BDU,
 * waits 100 ms, and discards the first successful direct read. Both verify
 * WHO_AM_I and register readback. Resume leaves FIFO and INT1 disabled: start
 * them explicitly for a new acquisition epoch. Restart filter/time history.
 * These calls do not switch the shared IMU/microphone supply or configure CPU
 * power management. Stop any GPIO event helper before a transition. Serialize
 * all calls in thread context; preserve sensor state for retries.
 * Failed transitions latch POWER_FAULT; acquisition returns -EPIPE until a
 * successful suspend/resume or init. Suspended acquisition returns -EACCES.
 * fifo_stop and disabling INT1 remain available but do not clear POWER_FAULT.
 * Repeating the current healthy state is a no-op, including resume while
 * already running. Keep the supply on; after a supply loss use init instead.
 */
int nrf54l15_lsm6ds3tr_suspend(struct nrf54l15_lsm6ds3tr *sensor);
int nrf54l15_lsm6ds3tr_resume(struct nrf54l15_lsm6ds3tr *sensor);

/* Single caller. Returns -EAGAIN until BOTH channels have fresh data.
 * A single 12-byte burst reads gyro then accel, with no cached fallback.
 * Output remains unchanged on any failure. Timestamp is host fetch time.
 * Returns -EBUSY while FIFO streaming is active or needs recovery. After
 * fifo_stop/resume, the first successful read is discarded and returns
 * -EAGAIN. Suspended/power-fault states reject acquisition before bus access.
 */
int nrf54l15_lsm6ds3tr_read(struct nrf54l15_lsm6ds3tr *sensor,
	struct nrf54l15_imu_sample *sample);

/* Read the independent temperature output registers when STATUS.TDA is set.
 * Returns -EAGAIN when no new data exists or when discarding the first value
 * after init/resume. Output is unchanged on all failures. Suspend/power faults
 * block access just as for motion reads. FIFO mode and FIFO faults do not block
 * temperature; this call never drains/repairs FIFO or consumes gyro/accel data.
 * Serialize with every other operation on this sensor. Timestamp is fetch time,
 * not a synchronized motion timestamp. Nominal scale is 25 C + raw / 256;
 * the datasheet permits +/-15 C offset. This is uncalibrated die temperature.
 */
int nrf54l15_lsm6ds3tr_temperature_read(struct nrf54l15_lsm6ds3tr *sensor,
	struct nrf54l15_temperature_sample *sample);

struct nrf54l15_lsm6ds3tr_fifo_sample {
	/* Sequence since fifo_start, modulo 2^32; first published frame is zero.
	 * This is a host count, not a sensor timestamp or guaranteed loss counter.
	 */
	uint32_t sequence;
	/* timestamp_us is FETCH time, not acquisition time. Feed a filter using
	 * the configured nominal sample period, not intervals between drain calls.
	 */
	struct nrf54l15_imu_sample sample;
};

/* Clear the FIFO and start continuous paired accel/gyro streaming at the
 * rate selected by init. The first pair is discarded after every start.
 * A start explicitly begins a new sequence/continuity epoch. Configuration
 * is read back before success. May retry after a failed start/read/stop.
 */
int nrf54l15_lsm6ds3tr_fifo_start(struct nrf54l15_lsm6ds3tr *sensor);

/* Clear and disable FIFO so direct output-register polling can resume.
 * Calling stop on an already stopped, healthy instance is a no-op.
 */
int nrf54l15_lsm6ds3tr_fifo_stop(struct nrf54l15_lsm6ds3tr *sensor);

/* Route only FIFO watermark to INT1, active-high push-pull as set by init.
 * Enable requires running power and an active, healthy FIFO; disable also
 * works after a FIFO fault. INT1_CTRL is written/read back. On failure routing is unknown: disable
 * the MCU callback and retry this operation explicitly; data reads still have
 * their normal validity checks. Routing survives fifo_start/stop and is cleared
 * by sensor init/reset or a successful power transition. Own INT1 exclusively;
 * no other interrupt sources are preserved. GPIO setup/waiting and clearing
 * FIFO backlog belong to the caller.
 */
int nrf54l15_lsm6ds3tr_fifo_int1(struct nrf54l15_lsm6ds3tr *sensor, bool enabled);

/* Read one oldest complete pair using six two-byte FIFO register reads,
 * checking status/pattern before and after. Single caller, thread context;
 * output unchanged on error.
 * -EAGAIN: no complete pair or initial pair discarded; retry later.
 * -EACCES: FIFO not started. -EOVERFLOW: full/overrun reported.
 * -EILSEQ: unexpected pattern. I2C errors are propagated.
 * Any bus/overrun/pattern fault latches: later reads return -EPIPE until
 * fifo_start (discard backlog) or fifo_stop succeeds. Never blindly retry a
 * partially completed FIFO read; it may already have consumed some words.
 */
int nrf54l15_lsm6ds3tr_fifo_read(struct nrf54l15_lsm6ds3tr *sensor,
	struct nrf54l15_lsm6ds3tr_fifo_sample *sample);

#ifdef __cplusplus
}
#endif
#endif
