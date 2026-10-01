/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <nrf54l15/openfloat_ble.h>
#include <nrf54l15/telemetry.h>

BUILD_ASSERT(CONFIG_BT_MAX_CONN == 1, "OpenFloat control state is for one peer");

static atomic_t ready;
static atomic_t stream_enabled = ATOMIC_INIT(1);
static atomic_t zero_requested;

/* The 48-bit UUID tail needs an explicit 64-bit type on 32-bit targets. */
#define OF_UUID_SERVICE BT_UUID_128_ENCODE(0x8f3f3b10, 0x0f5a, 0x4f4c, 0x9a2d, 0x000000000001ULL)
#define OF_UUID_LIVE BT_UUID_128_ENCODE(0x8f3f3b10, 0x0f5a, 0x4f4c, 0x9a2d, 0x000000000002ULL)
#define OF_UUID_CONTROL BT_UUID_128_ENCODE(0x8f3f3b10, 0x0f5a, 0x4f4c, 0x9a2d, 0x000000000003ULL)
static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(OF_UUID_SERVICE);
static struct bt_uuid_128 live_uuid = BT_UUID_INIT_128(OF_UUID_LIVE);
static struct bt_uuid_128 control_uuid = BT_UUID_INIT_128(OF_UUID_CONTROL);

static ssize_t write_control(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     const void *data, uint16_t length, uint16_t offset,
			     uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	if (offset != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
	}
	if (length < 4 || length > 5) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	enum nrf54l15_control_command command;
	if (nrf54l15_control_parse(data, length, &command) != 0) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	switch (command) {
	case NRF54L15_CONTROL_ZERO:
		atomic_set(&zero_requested, 1);
		break;
	case NRF54L15_CONTROL_START:
		atomic_set(&stream_enabled, 1);
		break;
	case NRF54L15_CONTROL_STOP:
		atomic_clear(&stream_enabled);
		break;
	}
	return length;
}

BT_GATT_SERVICE_DEFINE(openfloat_service,
	BT_GATT_PRIMARY_SERVICE(&service_uuid),
	BT_GATT_CHARACTERISTIC(&live_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&control_uuid.uuid,
		BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
		BT_GATT_PERM_WRITE, NULL, write_control, NULL)
);

static const struct bt_data advertising_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, OF_UUID_SERVICE),
};
static const struct bt_data scan_data[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static int advertise(void)
{
	return bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, advertising_data,
		ARRAY_SIZE(advertising_data), scan_data, ARRAY_SIZE(scan_data));
}

static void advertise_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	if (!atomic_get(&ready)) {
		return;
	}
	int err = advertise();
	if (err && err != -EALREADY) {
		printk("# BLE advertising failed: %d\n", err);
	}
}
static K_WORK_DEFINE(advertise_work, advertise_work_handler);

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(reason);
	atomic_set(&stream_enabled, 1);
}

static void recycled(void)
{
	/* A disconnect callback runs while the stack still owns a connection.
	 * With MAX_CONN=1, wait for recycled before starting connectable ads.
	 */
	(void)k_work_submit(&advertise_work);
}

BT_CONN_CB_DEFINE(openfloat_callbacks) = {
	.disconnected = disconnected,
	.recycled = recycled,
};

int nrf54l15_openfloat_ble_init(void)
{
	if (!atomic_get(&ready)) {
		int err = bt_enable(NULL);
		if (err) {
			return err;
		}
		atomic_set(&ready, 1);
	}
	/* Keep the enabled stack after an advertising failure so init can retry. */
	int err = advertise();
	return err == -EALREADY ? 0 : err;
}

int nrf54l15_openfloat_ble_send(uint32_t sequence, uint32_t dt_us,
	const struct nrf54l15_vec3 *accel, const struct nrf54l15_vec3 *gyro)
{
	if (!atomic_get(&ready)) {
		return -EAGAIN;
	}
	if (!atomic_get(&stream_enabled)) {
		return -EACCES;
	}
	uint8_t frame[NRF54L15_LIVE_FRAME_SIZE];
	int err = nrf54l15_live_encode(frame, sizeof(frame), sequence, dt_us, accel, gyro);
	if (err) {
		return err;
	}
	/* NULL lets the stack select and retain subscribed peers safely. No raw
	 * connection pointer is shared between the producer and disconnect path.
	 */
	return bt_gatt_notify(NULL, &openfloat_service.attrs[2], frame, sizeof(frame));
}

bool nrf54l15_openfloat_ble_take_zero_request(void)
{
	return atomic_set(&zero_requested, 0) != 0;
}
