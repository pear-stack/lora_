#ifndef SLEEP_H
#define SLEEP_H
#include <esp_sleep.h>

#define SLEEP_BETWEEN_MESSAGES 1
#define SLEEP_TIME_MS 20000
#define SLEEP_DELAY_MS 2000

void sleep_interrupt(uint8_t gpio, uint8_t mode);
void sleep_interrupt_mask(uint64_t mask, uint8_t mode);
void sleep_millis_2(uint64_t ms);
void sleep_seconds(uint32_t seconds);
void sleep_forever();

#endif