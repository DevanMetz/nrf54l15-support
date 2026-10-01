#ifndef TEST_ZEPHYR_BLUETOOTH_H
#define TEST_ZEPHYR_BLUETOOTH_H
#include <stddef.h>
#include <stdint.h>
struct bt_data { uint8_t type; uint8_t data_len; const uint8_t *data; };
struct bt_le_adv_param { int unused; };
#define BT_DATA_FLAGS 0x01
#define BT_DATA_UUID128_ALL 0x07
#define BT_DATA_NAME_COMPLETE 0x09
#define BT_LE_AD_GENERAL 0x02
#define BT_LE_AD_NO_BREDR 0x04
#define BT_DATA(_type, _data, _len) {(_type), (_len), (const uint8_t *)(_data)}
#define BT_DATA_BYTES(_type, ...) \
	BT_DATA(_type, ((const uint8_t[]){__VA_ARGS__}), sizeof((const uint8_t[]){__VA_ARGS__}))
#define BT_LE_ADV_CONN_FAST_1 (&(const struct bt_le_adv_param){0})
int bt_enable(void (*callback)(int));
int bt_le_adv_start(const struct bt_le_adv_param *params,
	const struct bt_data *ad, size_t ad_len, const struct bt_data *sd, size_t sd_len);
#endif
