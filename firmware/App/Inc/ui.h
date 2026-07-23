/* ui.h - clock-radio application: state machine, menus, rendering */
#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>

void ui_init(void);    /* init all peripherals/drivers + initial screen */
void ui_tick(void);    /* call continuously from the main loop */

/* software-interface hooks (console) - main-loop context only */
void ui_cmd_radio(bool on);
void ui_cmd_tune(uint32_t khz);
void ui_cmd_seek(bool up);
void ui_cmd_vol(uint8_t v);              /* 0..15 */
void ui_cmd_mute(bool m);
void ui_cmd_fmt24(bool f24);
void ui_cmd_alarm(int idx, int h, int m);
void ui_cmd_alarm_en(int idx, bool en);
void ui_cmd_alarm_sound(int idx, int snd);   /* 0-based */
void ui_cmd_alarm_vol(int idx, int vol);     /* 1..15 */
void ui_cmd_snooze(uint8_t mins);            /* 1..15 */
void ui_cmd_tz2(int off_hours);              /* 0 = off, else -12..+14 */
void ui_get_status(char *buf, int n);

#endif /* UI_H */
