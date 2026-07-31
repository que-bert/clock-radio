/* ds3231.h - DS3231 RTC over I2C */
#ifndef DS3231_H
#define DS3231_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t hour;    /* 0..23 (we keep 24h internally) */
    uint8_t min;
    uint8_t sec;
    uint8_t day;     /* 1..31 */
    uint8_t month;   /* 1..12 */
    uint8_t year;    /* 0..99 (20xx) */
} rtc_time_t;

void ds3231_init(void);
bool ds3231_get_time(rtc_time_t *t);
bool ds3231_set_time(const rtc_time_t *t);

/* Alarm 1, match on hours:minutes:00 each day */
bool ds3231_set_alarm(uint8_t hour, uint8_t min);
bool ds3231_alarm_enable(bool en);
bool ds3231_alarm_fired(void);   /* reads & returns A1F flag */
bool ds3231_clear_alarm(void);   /* clears A1F so INT pin releases */

#endif /* DS3231_H */
