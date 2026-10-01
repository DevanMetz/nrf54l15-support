/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <nrf54l15/openfloat_ble.h>
#include <nrf54l15/telemetry.h>

#define CHECK(expr) do { if (!(expr)) { \
	fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

extern const struct bt_gatt_service_static openfloat_service;
extern struct bt_conn_cb openfloat_callbacks;
static int enable_error, adv_error, notify_error;
static unsigned enable_calls, adv_calls, notify_calls, logged_errors;
static bool subscribed;
static struct k_work *pending_work;
static uint8_t last_frame[NRF54L15_LIVE_FRAME_SIZE];
static const uint8_t service_bytes[16] = {
	1, 0, 0, 0, 0, 0, 0x2d, 0x9a, 0x4c, 0x4f, 0x5a, 0x0f, 0x10, 0x3b, 0x3f, 0x8f,
};

int bt_enable(void (*callback)(int))
{
	CHECK(callback == NULL); /* Synchronous initialization. */
	++enable_calls;
	return enable_error;
}

int bt_le_adv_start(const struct bt_le_adv_param *params,
	const struct bt_data *ad, size_t ad_len, const struct bt_data *sd, size_t sd_len)
{
	CHECK(params != NULL && ad_len == 2 && sd_len == 1);
	CHECK(ad[0].type == BT_DATA_FLAGS && ad[0].data_len == 1 && ad[0].data[0] == 6);
	CHECK(ad[1].type == BT_DATA_UUID128_ALL && ad[1].data_len == 16);
	CHECK(memcmp(ad[1].data, service_bytes, 16) == 0);
	CHECK(sd[0].type == BT_DATA_NAME_COMPLETE);
	CHECK(sd[0].data_len == sizeof(CONFIG_BT_DEVICE_NAME) - 1);
	CHECK(memcmp(sd[0].data, CONFIG_BT_DEVICE_NAME, sd[0].data_len) == 0);
	++adv_calls;
	return adv_error;
}

int bt_gatt_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	const void *data, uint16_t len)
{
	CHECK(conn == NULL && attr == &openfloat_service.attrs[2]);
	CHECK(len == sizeof(last_frame));
	memcpy(last_frame, data, len);
	++notify_calls;
	return subscribed ? notify_error : -ENOTCONN;
}

int k_work_submit(struct k_work *work)
{
	CHECK(pending_work == NULL || pending_work == work);
	int result = pending_work == NULL ? 1 : 0;
	pending_work = work;
	return result;
}

static void run_work(void)
{
	CHECK(pending_work != NULL);
	struct k_work *work = pending_work;
	pending_work = NULL;
	work->handler(work);
}

int printk(const char *format, ...)
{
	CHECK(strstr(format, "advertising failed") != NULL);
	++logged_errors;
	return 0;
}

static ssize_t control(const void *data, uint16_t len, uint16_t offset, uint8_t flags)
{
	const struct bt_gatt_attr *attr = &openfloat_service.attrs[5];
	CHECK(attr->write != NULL && attr->perm == BT_GATT_PERM_WRITE);
	return attr->write(NULL, attr, data, len, offset, flags);
}

int main(void)
{
	struct nrf54l15_vec3 accel = {NRF54L15_GRAVITY_MPS2, 0, 0};
	struct nrf54l15_vec3 gyro = {0, 0, 0};
	CHECK(openfloat_service.attr_count == 6);
	const struct bt_uuid_128 *uuid = openfloat_service.attrs[0].user_data;
	CHECK(memcmp(uuid->val, service_bytes, 16) == 0);
	uuid = (const struct bt_uuid_128 *)openfloat_service.attrs[2].uuid;
	CHECK(uuid->val[0] == 2 && memcmp(uuid->val + 1, service_bytes + 1, 15) == 0);
	uuid = (const struct bt_uuid_128 *)openfloat_service.attrs[5].uuid;
	CHECK(uuid->val[0] == 3 && memcmp(uuid->val + 1, service_bytes + 1, 15) == 0);
	CHECK(!nrf54l15_openfloat_ble_take_zero_request());
	CHECK(nrf54l15_openfloat_ble_send(8, 2404, &accel, &gyro) == -EAGAIN);
	openfloat_callbacks.recycled();
	run_work();
	CHECK(adv_calls == 0);

	/* A failed enable can retry. A failed advertisement must not re-enable BT. */
	enable_error = -EIO;
	CHECK(nrf54l15_openfloat_ble_init() == -EIO);
	CHECK(enable_calls == 1 && adv_calls == 0);
	CHECK(nrf54l15_openfloat_ble_send(8, 2404, &accel, &gyro) == -EAGAIN);
	enable_error = 0;
	adv_error = -ENOMEM;
	CHECK(nrf54l15_openfloat_ble_init() == -ENOMEM);
	CHECK(enable_calls == 2 && adv_calls == 1);
	adv_error = 0;
	CHECK(nrf54l15_openfloat_ble_init() == 0);
	CHECK(enable_calls == 2 && adv_calls == 2);
	adv_error = -EALREADY;
	CHECK(nrf54l15_openfloat_ble_init() == 0);
	CHECK(enable_calls == 2 && adv_calls == 3);

	CHECK(nrf54l15_openfloat_ble_send(8, 2404, &accel, &gyro) == -ENOTCONN);
	CHECK(notify_calls == 1);
	CHECK(control("stop", 4, 0, 0) == 4);
	subscribed = true;
	CHECK(nrf54l15_openfloat_ble_send(8, 2404, &accel, &gyro) == -EACCES);
	CHECK(notify_calls == 1); /* A CCC change does not override explicit stop. */
	CHECK(control("start", 5, 1, 0) == -BT_ATT_ERR_INVALID_OFFSET);
	CHECK(control("start", 5, 0, BT_GATT_WRITE_FLAG_PREPARE) == -BT_ATT_ERR_NOT_SUPPORTED);
	CHECK(control("startx", 6, 0, 0) == -BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(control("sta", 3, 0, 0) == -BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	CHECK(control("STARt", 5, 0, 0) == -BT_ATT_ERR_VALUE_NOT_ALLOWED);
	CHECK(control("star\0", 5, 0, 0) == -BT_ATT_ERR_VALUE_NOT_ALLOWED);
	CHECK(control(NULL, 5, 0, 0) == -BT_ATT_ERR_VALUE_NOT_ALLOWED);
	CHECK(nrf54l15_openfloat_ble_send(8, 2404, &accel, &gyro) == -EACCES);
	CHECK(control("start", 5, 0, 0) == 5);
	CHECK(nrf54l15_openfloat_ble_send(65544, 70000, &accel, &gyro) == 0);
	const uint8_t expected[20] = {
		'O', 'F', 1, 1, 8, 0, 255, 255, 0xe8, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	};
	CHECK(memcmp(last_frame, expected, sizeof(expected)) == 0);
	CHECK(control("zero", 4, 0, 0) == 4 && control("zero", 4, 0, 0) == 4);
	CHECK(nrf54l15_openfloat_ble_take_zero_request());
	CHECK(!nrf54l15_openfloat_ble_take_zero_request());
	CHECK(control("zero\0", 5, 0, 0) == -BT_ATT_ERR_VALUE_NOT_ALLOWED);
	CHECK(!nrf54l15_openfloat_ble_take_zero_request());
	unsigned sent = notify_calls;
	gyro.x = NAN;
	CHECK(nrf54l15_openfloat_ble_send(16, 2404, &accel, &gyro) == -EINVAL);
	CHECK(notify_calls == sent);
	gyro.x = 0;
	notify_error = -ENOMEM;
	CHECK(nrf54l15_openfloat_ble_send(16, 2404, &accel, &gyro) == -ENOMEM);
	notify_error = 0;

	/* Disconnect restores stream policy. Advertising waits for recycled/work. */
	CHECK(control("stop", 4, 0, 0) == 4);
	unsigned advertised = adv_calls;
	subscribed = false;
	openfloat_callbacks.disconnected(NULL, 0x13);
	CHECK(pending_work == NULL && adv_calls == advertised);
	CHECK(nrf54l15_openfloat_ble_send(24, 2404, &accel, &gyro) == -ENOTCONN);
	openfloat_callbacks.recycled();
	openfloat_callbacks.recycled();
	CHECK(adv_calls == advertised);
	adv_error = 0;
	run_work();
	CHECK(adv_calls == advertised + 1 && logged_errors == 0);
	subscribed = true;
	CHECK(nrf54l15_openfloat_ble_send(32, 2404, &accel, &gyro) == 0);
	adv_error = -EALREADY;
	openfloat_callbacks.recycled();
	run_work();
	CHECK(logged_errors == 0);
	adv_error = -ENOMEM;
	openfloat_callbacks.recycled();
	run_work();
	CHECK(logged_errors == 1);
	puts("BLE lifecycle and control tests passed");
	return 0;
}
