/* console.c - line-oriented command console over USB CDC.
 *
 * This is the project's *software* user interface (rubric: set time/alarm,
 * tune, volume from software). RX bytes arrive in USB interrupt context and
 * are only queued here; parsing and command execution (which touch the I2C
 * bus) happen in console_tick() from the main loop, so they can't race the
 * UI's own I2C transfers.
 */
#include "console.h"
#include "ui.h"
#include "ds3231.h"
#include "audio.h"
#include "rda5807.h"
#include "input.h"
#include "settings.h"
#include "usbd_cdc_if.h"
#include "bsp.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/* ---- RX ring buffer (IRQ producer, main-loop consumer) ---- */
#define RB_SIZE 256u
static volatile uint8_t  rb[RB_SIZE];
static volatile uint16_t rb_head, rb_tail;

void console_rx(const uint8_t *buf, uint32_t len)
{
    while (len--) {
        uint16_t next = (rb_head + 1) % RB_SIZE;
        if (next == rb_tail) break;          /* full - drop the rest */
        rb[rb_head] = *buf++;
        rb_head = next;
    }
}

/* ---- TX ---- */
static uint32_t cputs_dbg_max;         /* worst cputs busy-wait, ms */
static uint32_t con_dur_max;           /* worst console_tick duration, ms */

static void cputs(const char *s)
{
    uint16_t len = (uint16_t)strlen(s);
    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t *)s, len) == 1 /* USBD_BUSY */) {
        if (HAL_GetTick() - t0 > 50) break;  /* host gone - don't hang */
    }
    uint32_t dt = HAL_GetTick() - t0;
    if (dt > cputs_dbg_max) cputs_dbg_max = dt;
}

/* ---- helpers ---- */
static const char *next_tok(char **p)
{
    char *s = *p;
    while (*s == ' ') s++;
    if (*s == '\0') { *p = s; return NULL; }
    char *tok = s;
    while (*s && *s != ' ') s++;
    if (*s) *s++ = '\0';
    *p = s;
    return tok;
}

static bool parse_hhmm(const char *s, int *h, int *m, int *sec)
{
    char *e;
    long hh = strtol(s, &e, 10);
    if (*e != ':') return false;
    long mm = strtol(e + 1, &e, 10);
    long ss = 0;
    if (*e == ':') ss = strtol(e + 1, &e, 10);
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 59)
        return false;
    *h = (int)hh; *m = (int)mm;
    if (sec) *sec = (int)ss;
    return true;
}

/* parse a day-of-week spec into a mask (bit0=Sun..bit6=Sat).
 * accepts: daily/every/all, weekdays, weekends, a numeric mask 0..127,
 * or a comma list of two-letter codes su,mo,tu,we,th,fr,sa. */
static bool parse_days(const char *s, uint8_t *mask)
{
    static const char *ab[7] = {"su","mo","tu","we","th","fr","sa"};
    if (!strcmp(s,"daily") || !strcmp(s,"every") || !strcmp(s,"everyday") ||
        !strcmp(s,"all"))                       { *mask = SET_DOW_DAILY;    return true; }
    if (!strcmp(s,"weekdays"))                  { *mask = SET_DOW_WEEKDAYS; return true; }
    if (!strcmp(s,"weekends"))                  { *mask = SET_DOW_WEEKENDS; return true; }
    if (s[0] >= '0' && s[0] <= '9') {           /* raw numeric mask */
        long v = strtol(s, NULL, 0);
        if (v < 0 || v > 127) return false;
        *mask = (uint8_t)v; return true;
    }
    uint8_t m = 0;
    for (const char *p = s; *p; ) {
        int found = -1;
        for (int k = 0; k < 7; k++)
            if (p[0] == ab[k][0] && p[1] == ab[k][1]) { found = k; break; }
        if (found < 0) return false;
        m |= (uint8_t)(1u << found);
        p += 2;
        if (*p == ',') p++;
        else if (*p)   return false;            /* junk after a code */
    }
    if (!m) return false;
    *mask = m; return true;
}

static void do_status(void)
{
    char buf[512];
    ui_get_status(buf, sizeof buf);
    cputs(buf);
}

static const char *HELP =
    "commands:\r\n"
    "  status                    show everything\r\n"
    "  time HH:MM[:SS]           set clock\r\n"
    "  date YYYY-MM-DD           set date\r\n"
    "  12h | 24h                 display format\r\n"
    "  tz2 off | tz2 +N | -N     second time zone\r\n"
    "  alarm                     list alarms\r\n"
    "  alarm N HH:MM             set alarm N (1-4)\r\n"
    "  alarm N on|off            enable/disable\r\n"
    "  alarm N sound 1-4         Classic/Buzzer/Soft/Klaxon\r\n"
    "  alarm N vol 1-15\r\n"
    "  alarm N days SPEC         daily|weekdays|weekends|su,mo,tu,we,th,fr,sa\r\n"
    "  snooze 1-15               snooze minutes\r\n"
    "  radio on|off\r\n"
    "  tune MHZ                  e.g. tune 101.9\r\n"
    "  seek [up|down]\r\n"
    "  vol 0-15\r\n"
    "  mute on|off\r\n"
    "  usbaudio on|off           enable/disable the USB speaker\r\n"
    "  rdsraw                    dump one RDS frame (debug)\r\n"
    "  rdsmsg                    show learned scroll message (debug)\r\n"
    "  rdsfr                     dump RDS frame registry (debug)\r\n"
    "  dfu                       reboot into USB-DFU bootloader\r\n";

static void exec_line(char *line)
{
    char *p = line;
    const char *cmd = next_tok(&p);
    if (!cmd) return;

    if (!strcmp(cmd, "help")) { cputs(HELP); return; }
    if (!strcmp(cmd, "status")) { do_status(); return; }

    if (!strcmp(cmd, "time")) {
        const char *a = next_tok(&p);
        int h, m, s;
        rtc_time_t t;
        if (a && parse_hhmm(a, &h, &m, &s) && ds3231_get_time(&t)) {
            t.hour = (uint8_t)h; t.min = (uint8_t)m; t.sec = (uint8_t)s;
            ds3231_set_time(&t);
            cputs("ok\r\n");
        } else cputs("err: time HH:MM[:SS]\r\n");
        return;
    }
    if (!strcmp(cmd, "date")) {
        const char *a = next_tok(&p);
        char *e;
        rtc_time_t t;
        if (a) {
            long y = strtol(a, &e, 10);
            if (*e == '-') {
                long mo = strtol(e + 1, &e, 10);
                if (*e == '-') {
                    long d = strtol(e + 1, &e, 10);
                    if (y >= 2000 && y <= 2099 && mo >= 1 && mo <= 12 &&
                        d >= 1 && d <= 31 && ds3231_get_time(&t)) {
                        t.year = (uint8_t)(y - 2000);
                        t.month = (uint8_t)mo; t.day = (uint8_t)d;
                        ds3231_set_time(&t);
                        cputs("ok\r\n");
                        return;
                    }
                }
            }
        }
        cputs("err: date YYYY-MM-DD\r\n");
        return;
    }
    if (!strcmp(cmd, "12h")) { ui_cmd_fmt24(false); cputs("ok\r\n"); return; }
    if (!strcmp(cmd, "24h")) { ui_cmd_fmt24(true);  cputs("ok\r\n"); return; }

    if (!strcmp(cmd, "tz2")) {
        const char *a = next_tok(&p);
        if (a && !strcmp(a, "off")) { ui_cmd_tz2(0); cputs("ok\r\n"); return; }
        if (a) {
            long o = strtol(a, NULL, 10);
            if (o >= -12 && o <= 14 && o != 0) {
                ui_cmd_tz2((int)o); cputs("ok\r\n"); return;
            }
        }
        cputs("err: tz2 off | tz2 -12..+14\r\n");
        return;
    }

    if (!strcmp(cmd, "alarm")) {
        const char *a = next_tok(&p);
        if (!a) { do_status(); return; }
        int idx = atoi(a) - 1;
        const char *v = next_tok(&p);
        if (idx < 0 || idx > 3 || !v) { cputs("err: alarm 1-4 ...\r\n"); return; }
        if (!strcmp(v, "on"))  { ui_cmd_alarm_en(idx, true);  cputs("ok\r\n"); return; }
        if (!strcmp(v, "off")) { ui_cmd_alarm_en(idx, false); cputs("ok\r\n"); return; }
        if (!strcmp(v, "sound")) {
            const char *w = next_tok(&p);
            int s = w ? atoi(w) : 0;
            if (s >= 1 && s <= AUDIO_N_SOUNDS) {
                ui_cmd_alarm_sound(idx, s - 1); cputs("ok\r\n");
            } else cputs("err: sound 1-4\r\n");
            return;
        }
        if (!strcmp(v, "vol")) {
            const char *w = next_tok(&p);
            int vv = w ? atoi(w) : 0;
            if (vv >= 1 && vv <= 15) { ui_cmd_alarm_vol(idx, vv); cputs("ok\r\n"); }
            else cputs("err: vol 1-15\r\n");
            return;
        }
        if (!strcmp(v, "days")) {
            const char *w = next_tok(&p);
            uint8_t mask;
            if (w && parse_days(w, &mask)) {
                ui_cmd_alarm_days(idx, mask); cputs("ok\r\n");
            } else cputs("err: days daily|weekdays|weekends|su,mo,tu,we,th,fr,sa\r\n");
            return;
        }
        int h, m;
        if (parse_hhmm(v, &h, &m, NULL)) {
            ui_cmd_alarm(idx, h, m); cputs("ok\r\n");
        } else cputs("err: alarm N HH:MM|on|off|sound S|vol V\r\n");
        return;
    }
    if (!strcmp(cmd, "snooze")) {
        const char *a = next_tok(&p);
        int v = a ? atoi(a) : 0;
        if (v >= 1 && v <= 15) { ui_cmd_snooze((uint8_t)v); cputs("ok\r\n"); }
        else cputs("err: snooze 1-15\r\n");
        return;
    }
    if (!strcmp(cmd, "radio")) {
        const char *a = next_tok(&p);
        if (a && !strcmp(a, "on"))  { ui_cmd_radio(true);  cputs("ok\r\n"); return; }
        if (a && !strcmp(a, "off")) { ui_cmd_radio(false); cputs("ok\r\n"); return; }
        cputs("err: radio on|off\r\n");
        return;
    }
    if (!strcmp(cmd, "tune")) {
        const char *a = next_tok(&p);
        char *e;
        if (a) {
            long mhz = strtol(a, &e, 10);
            long tenth = 0;
            if (*e == '.') tenth = strtol(e + 1, &e, 10) % 10;
            uint32_t khz = (uint32_t)(mhz * 1000 + tenth * 100);
            if (khz >= 87500 && khz <= 108000) {
                ui_cmd_tune(khz); cputs("ok\r\n"); return;
            }
        }
        cputs("err: tune 87.5-108.0\r\n");
        return;
    }
    if (!strcmp(cmd, "seek")) {
        const char *a = next_tok(&p);
        ui_cmd_seek(!(a && !strcmp(a, "down")));
        cputs("ok\r\n");
        return;
    }
    if (!strcmp(cmd, "vol")) {
        const char *a = next_tok(&p);
        int v = a ? atoi(a) : -1;
        if (v >= 0 && v <= 15) { ui_cmd_vol((uint8_t)v); cputs("ok\r\n"); }
        else cputs("err: vol 0-15\r\n");
        return;
    }
    if (!strcmp(cmd, "mute")) {
        const char *a = next_tok(&p);
        if (a && !strcmp(a, "on"))  { ui_cmd_mute(true);  cputs("ok\r\n"); return; }
        if (a && !strcmp(a, "off")) { ui_cmd_mute(false); cputs("ok\r\n"); return; }
        cputs("err: mute on|off\r\n");
        return;
    }
    if (!strcmp(cmd, "usbaudio")) {
        const char *a = next_tok(&p);
        if (a && !strcmp(a, "on"))  { ui_cmd_usb_audio(true);  cputs("ok\r\n"); return; }
        if (a && !strcmp(a, "off")) { ui_cmd_usb_audio(false); cputs("ok\r\n"); return; }
        cputs("err: usbaudio on|off\r\n");
        return;
    }
    if (!strcmp(cmd, "rdsraw")) {
        /* debug: dump one RDS frame (regs 0x0A..0x0F) and decode the basics */
        uint8_t r[12];
        if (HAL_I2C_Master_Receive(&hi2c2, ADDR_RDA5807, r, 12, 50) != HAL_OK) {
            cputs("err: i2c\r\n"); return;
        }
        uint16_t ra = (uint16_t)((r[0] << 8) | r[1]);
        uint16_t rb = (uint16_t)((r[2] << 8) | r[3]);
        uint16_t bb = (uint16_t)((r[6] << 8) | r[7]);
        uint16_t bd = (uint16_t)((r[10] << 8) | r[11]);
        char c0 = (char)(bd >> 8), c1 = (char)(bd & 0xFF);
        char buf[128];
        snprintf(buf, sizeof buf,
                 "rdsr=%c bler=%u/%u grp=%u pi=%u d='%c%c'  a=%04x b=%04x B=%04x D=%04x\r\n",
                 (ra & 0x8000) ? 'Y' : 'n', (rb >> 2) & 3, rb & 3,
                 (bb >> 12) & 0x0F, bb & 3,
                 (c0 >= 0x20 && c0 < 0x7F) ? c0 : '.',
                 (c1 >= 0x20 && c1 < 0x7F) ? c1 : '.',
                 ra, rb, bb, bd);
        cputs(buf);
        return;
    }
    if (!strcmp(cmd, "rdsmsg")) {
        /* debug: show the learned scrolling message */
        char buf[160];
        rda_rds_msg(buf, sizeof buf);
        cputs(buf);
        cputs("\r\n");
        return;
    }
    if (!strcmp(cmd, "usbstat")) {
        uint32_t pkts, dout, isoinc; uint16_t lastn, gap, wr; int16_t smp; uint8_t fl;
        audio_usb_stats(&pkts, &dout, &isoinc, &lastn, &gap, &wr, &smp, &fl);
        char b[160];
        snprintf(b, sizeof b,
                 "usb dout=%lu iso_inc=%lu wrote=%lu lastn=%u smp0=%d gap=%u wr=%u "
                 "dac=%u alt=%u blk=%u amp=%u\r\n",
                 (unsigned long)dout, (unsigned long)isoinc, (unsigned long)pkts,
                 lastn, smp, gap, wr,
                 fl & 1, (fl >> 1) & 1, (fl >> 2) & 1, (fl >> 3) & 1);
        cputs(b);
        return;
    }
    if (!strcmp(cmd, "btnstat")) {
        /* debug: button edges, worst input-poll gap, and blocker durations */
        extern uint32_t oled_dbg_max;
        char b[96];
        snprintf(b, sizeof b,
                 "edges=%lu maxgap=%lums oled=%lums cputs=%lums rtc=%lums "
                 "ui=%lums con=%lums rend=%lums sec=%lums\r\n",
                 (unsigned long)input_dbg_edges(),
                 (unsigned long)ui_dbg_ingap(),
                 (unsigned long)oled_dbg_max,
                 (unsigned long)cputs_dbg_max,
                 (unsigned long)ui_dbg_rtcmax(),
                 (unsigned long)ui_dbg_durmax(),
                 (unsigned long)con_dur_max,
                 (unsigned long)ui_dbg_rendmax(),
                 (unsigned long)ui_dbg_secmax());
        oled_dbg_max = 0; cputs_dbg_max = 0; con_dur_max = 0;
        cputs(b);
        return;
    }
    if (!strcmp(cmd, "dfu")) {
        cputs("entering DFU bootloader (reflash with dfu-util)...\r\n");
        HAL_Delay(100);              /* let the reply flush over CDC */
        dfu_jump();
        return;                      /* not reached */
    }
    if (!strcmp(cmd, "rdsfr")) {
        /* debug: dump the frame registry with order evidence */
        char buf[48];
        for (int i = 0; rda_rds_frame(i, buf, sizeof buf); i++) {
            cputs(buf);
            cputs("\r\n");
        }
        return;
    }
    cputs("err: unknown command ('help')\r\n");
}

void console_tick(void)
{
    static char line[80];
    static uint8_t pos;
    uint32_t t0 = HAL_GetTick();

    while (rb_tail != rb_head) {
        char c = (char)rb[rb_tail];
        rb_tail = (rb_tail + 1) % RB_SIZE;
        if (c == '\r' || c == '\n') {
            if (pos) {
                line[pos] = '\0';
                for (uint8_t i = 0; i < pos; i++)
                    line[i] = (char)tolower((unsigned char)line[i]);
                exec_line(line);
                pos = 0;
            }
        } else if (pos < sizeof line - 1) {
            line[pos++] = c;
        }
    }
    uint32_t dt = HAL_GetTick() - t0;
    if (dt > con_dur_max) con_dur_max = dt;
}
