/* settings.h - persistent user settings in the last internal-flash page */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

#define SET_N_ALARMS 4

typedef struct {
    uint16_t magic;              /* filled in by settings_save */
    uint8_t  version;            /* filled in by settings_save */
    uint8_t  volume;             /* 0..15 */
    uint32_t radio_khz;
    uint8_t  alarm_h[SET_N_ALARMS];
    uint8_t  alarm_m[SET_N_ALARMS];
    uint8_t  alarm_en;           /* bitmask, bit i = alarm i enabled */
    uint8_t  alarm_vol[SET_N_ALARMS];    /* 1..15, per alarm */
    uint8_t  alarm_sound[SET_N_ALARMS];  /* audio.c sound index, per alarm */
    uint8_t  snooze_len;         /* minutes */
    uint8_t  flags;              /* bit1=fmt24 bit2=radio_on */
    uint8_t  contrast;
    int8_t   tz2;                /* 2nd time zone offset hours, 0 = off */
    uint8_t  pad[2];             /* keep zeroed */
    uint8_t  checksum;           /* filled in by settings_save */
} settings_t;                    /* exactly one 32-byte flash slot */

#define SET_F_FMT24     0x02
#define SET_F_RADIO_ON  0x04

bool settings_load(settings_t *out);      /* false = nothing stored yet */
bool settings_save(const settings_t *in); /* ~170us, ~40ms every 128th call */

#endif /* SETTINGS_H */
