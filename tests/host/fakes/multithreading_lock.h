#ifndef TEST_MULTITHREADING_LOCK_H
#define TEST_MULTITHREADING_LOCK_H
#include <stdint.h>
int multithreading_lock_acquire(uint32_t timeout);
void multithreading_lock_release(void);
#endif
