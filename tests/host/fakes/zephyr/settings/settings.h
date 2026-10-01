#ifndef TEST_ZEPHYR_SETTINGS_H
#define TEST_ZEPHYR_SETTINGS_H
#include <stddef.h>
typedef ptrdiff_t ssize_t;
int settings_subsys_init(void);
ssize_t settings_load_one(const char *name, void *buf, size_t buf_len);
int settings_save_one(const char *name, const void *value, size_t val_len);
int settings_delete(const char *name);
#endif
