#ifndef TEST_ZEPHYR_IRQ_H
#define TEST_ZEPHYR_IRQ_H
unsigned int irq_lock(void);
void irq_unlock(unsigned int key);
#endif
