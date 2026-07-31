/* ds3231.c - DS3231 RTC driver */
#include "ds3231.h"
#include "bsp.h"

#define REG_TIME    0x00
#define REG_ALARM1  0x07
#define REG_CONTROL 0x0E
#define REG_STATUS  0x0F

static uint8_t bcd2dec(uint8_t b) { return (b >> 4) * 10 + (b & 0x0F); }
static uint8_t dec2bcd(uint8_t d) { return ((d / 10) << 4) | (d % 10); }

static bool rd(uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(&hi2c2, ADDR_DS3231, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, n, 50) == HAL_OK;
}
static bool wr(uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Write(&hi2c2, ADDR_DS3231, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, n, 50) == HAL_OK;
}

void ds3231_init(void)
{
    /* INTCN=1 (INT pin = alarm interrupt, not square wave); alarms off */
    uint8_t ctrl = 0x04;
    wr(REG_CONTROL, &ctrl, 1);
    ds3231_clear_alarm();
}

bool ds3231_get_time(rtc_time_t *t)
{
    uint8_t b[7];
    if (!rd(REG_TIME, b, 7)) return false;
    t->sec   = bcd2dec(b[0] & 0x7F);
    t->min   = bcd2dec(b[1] & 0x7F);
    t->hour  = bcd2dec(b[2] & 0x3F);      /* assume 24h mode stored */
    t->day   = bcd2dec(b[4] & 0x3F);
    t->month = bcd2dec(b[5] & 0x1F);
    t->year  = bcd2dec(b[6]);
    return true;
}

bool ds3231_set_time(const rtc_time_t *t)
{
    uint8_t b[7];
    b[0] = dec2bcd(t->sec);
    b[1] = dec2bcd(t->min);
    b[2] = dec2bcd(t->hour);              /* bit6=0 -> 24h mode */
    b[3] = 1;                             /* day-of-week, unused */
    b[4] = dec2bcd(t->day);
    b[5] = dec2bcd(t->month);
    b[6] = dec2bcd(t->year);
    return wr(REG_TIME, b, 7);
}

bool ds3231_set_alarm(uint8_t hour, uint8_t min)
{
    /* A1: match hours+minutes, seconds=0; A1M4=0,A1M3=0,A1M2=0,A1M1=0 */
    uint8_t b[4];
    b[0] = dec2bcd(0);                    /* seconds, A1M1=0 */
    b[1] = dec2bcd(min);                  /* A1M2=0 */
    b[2] = dec2bcd(hour);                 /* A1M3=0, 24h */
    b[3] = 0x80;                          /* A1M4=1 -> ignore day/date */
    return wr(REG_ALARM1, b, 4);
}

bool ds3231_alarm_enable(bool en)
{
    uint8_t ctrl;
    if (!rd(REG_CONTROL, &ctrl, 1)) return false;
    ctrl |= 0x04;                         /* INTCN */
    if (en) ctrl |= 0x01; else ctrl &= ~0x01;   /* A1IE */
    return wr(REG_CONTROL, &ctrl, 1);
}

bool ds3231_alarm_fired(void)
{
    uint8_t st;
    if (!rd(REG_STATUS, &st, 1)) return false;
    return (st & 0x01) != 0;              /* A1F */
}

bool ds3231_clear_alarm(void)
{
    uint8_t st;
    if (!rd(REG_STATUS, &st, 1)) return false;
    st &= ~0x01;                          /* clear A1F */
    return wr(REG_STATUS, &st, 1);
}
