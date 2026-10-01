#ifndef TEST_ZEPHYR_BT_UUID_H
#define TEST_ZEPHYR_BT_UUID_H
#include <stdint.h>
struct bt_uuid { uint8_t type; };
struct bt_uuid_128 { struct bt_uuid uuid; uint8_t val[16]; };
#define TEST_UUID_BYTE(value, shift) ((uint8_t)((value) >> (shift)))
#define BT_UUID_128_ENCODE(w32, w1, w2, w3, w48) \
	TEST_UUID_BYTE(w48, 0), TEST_UUID_BYTE(w48, 8), TEST_UUID_BYTE(w48, 16), \
	TEST_UUID_BYTE(w48, 24), TEST_UUID_BYTE(w48, 32), TEST_UUID_BYTE(w48, 40), \
	TEST_UUID_BYTE(w3, 0), TEST_UUID_BYTE(w3, 8), \
	TEST_UUID_BYTE(w2, 0), TEST_UUID_BYTE(w2, 8), \
	TEST_UUID_BYTE(w1, 0), TEST_UUID_BYTE(w1, 8), \
	TEST_UUID_BYTE(w32, 0), TEST_UUID_BYTE(w32, 8), \
	TEST_UUID_BYTE(w32, 16), TEST_UUID_BYTE(w32, 24)
#define BT_UUID_INIT_128(...) {{2}, {__VA_ARGS__}}
#endif
