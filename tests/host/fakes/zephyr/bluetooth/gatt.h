#ifndef TEST_ZEPHYR_BT_GATT_H
#define TEST_ZEPHYR_BT_GATT_H
#include <stddef.h>
#include <stdint.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#ifdef _WIN32
typedef ptrdiff_t ssize_t;
#else
#include <sys/types.h>
#endif
struct bt_gatt_attr;
typedef ssize_t (*bt_gatt_attr_write_func_t)(struct bt_conn *, const struct bt_gatt_attr *,
	const void *, uint16_t, uint16_t, uint8_t);
struct bt_gatt_attr {
	const struct bt_uuid *uuid;
	bt_gatt_attr_write_func_t write;
	uint16_t perm;
	const void *user_data;
};
struct bt_gatt_service_static { const struct bt_gatt_attr *attrs; size_t attr_count; };
#define BT_GATT_WRITE_FLAG_PREPARE 1
#define BT_GATT_CHRC_WRITE_WITHOUT_RESP 0x04
#define BT_GATT_CHRC_WRITE 0x08
#define BT_GATT_CHRC_NOTIFY 0x10
#define BT_GATT_PERM_NONE 0
#define BT_GATT_PERM_READ 1
#define BT_GATT_PERM_WRITE 2
#define BT_GATT_ERR(error) (-(error))
/* Preserve real Zephyr attribute positions; notification values are index 2. */
#define BT_GATT_PRIMARY_SERVICE(_uuid) {NULL, NULL, BT_GATT_PERM_READ, (_uuid)}
#define BT_GATT_CHARACTERISTIC(_uuid, _properties, _perm, _read, _write, _data) \
	{NULL, NULL, BT_GATT_PERM_READ, NULL}, {(_uuid), (_write), (_perm), (_data)}
#define BT_GATT_CCC(_callback, _perm) {NULL, NULL, (_perm), NULL}
#define BT_GATT_SERVICE_DEFINE(name, ...) \
	static const struct bt_gatt_attr name##_attrs[] = {__VA_ARGS__}; \
	const struct bt_gatt_service_static name = {name##_attrs, \
		sizeof(name##_attrs) / sizeof(name##_attrs[0])}
int bt_gatt_notify(struct bt_conn *conn, const struct bt_gatt_attr *attr,
	const void *data, uint16_t len);
#endif
