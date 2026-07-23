/* ui.c - clock-radio application logic & rendering
 *
 * Interaction model (single rotary encoder + push button):
 *   HOME:   rotate = volume,  click = mute,  double = menu,  long = radio on/off
 *   MENU:   rotate = scroll,  click = select, long/Back = up, double = home
 *   EDIT:   rotate = value,    click = next field, long = cancel
 *   ALARM:  click = snooze,    long = dismiss
 */
#include "ui.h"
#include "bsp.h"
#include "oled.h"
#include "ds3231.h"
#include "rda5807.h"
#include "audio.h"
#include "input.h"
#include "settings.h"
#include <stdio.h>
#include <string.h>

/* ---------------- application state ---------------- */
typedef enum { ST_HOME, ST_MENU, ST_SET_TIME, ST_SET_DATE, ST_SET_ALARM,
               ST_TUNE, ST_DIAG, ST_RING } state_t;

static state_t st = ST_HOME;

static rtc_time_t now;
static bool     fmt24      = true;
#define N_ALARMS SET_N_ALARMS
static uint8_t  alarm_h[N_ALARMS] = {7, 7, 7, 7};
static uint8_t  alarm_m[N_ALARMS] = {30, 30, 30, 30};
static uint8_t  alarm_en_mask = 0;       /* bit i = alarm i enabled */
static uint8_t  cur_alarm;               /* alarm being edited */
static uint8_t  alarm_vol[N_ALARMS] = {12, 12, 12, 12};  /* 1..15, per alarm */
static uint8_t  alarm_snd[N_ALARMS] = {0, 0, 0, 0};      /* sound table index */
static uint8_t  ring_alarm;              /* alarm now ringing/snoozed */
static uint32_t preview_until;           /* tone preview deadline, 0 = idle */
static int8_t   tz2_off;                 /* 2nd time zone, hours; 0 = off */

/* in-menu value editing: click a value row, rotate, click to accept */
enum { EDIT_NONE, EDIT_SNOOZE, EDIT_TZ2 };
static uint8_t  menu_editing;
static uint8_t  volume     = 6;          /* 0..15 */
static bool     muted      = false;
static bool     radio_on   = false;
static uint32_t radio_khz  = 101900;
static char     rds[9]     = "        ";
static uint8_t  contrast   = 0x80;

/* settings persistence: mark dirty on change, write 3s after the last one
 * so a volume-knob spin becomes one flash write, not thirty */
static bool     cfg_dirty;
static uint32_t cfg_at;

/* deferred power-on tune: the RDA5807's crystal needs a few hundred ms
 * after enable before a TUNE can lock; tuning during ui_init fails
 * silently and leaves static until the user toggles the radio */
static uint32_t resume_tune_at;          /* 0 = nothing pending */

/* alarm runtime */
static bool     ringing    = false;
static bool     snoozed    = false;
static uint32_t snooze_at  = 0;
static uint8_t  snooze_len = 9;          /* minutes */
static int8_t   fired_min  = -1;         /* minute fired_mask belongs to */
static uint8_t  fired_mask;              /* alarms already fired this minute */

/* edit scratch */
static rtc_time_t edit_t;
static uint8_t  edit_field;              /* time: 0=hour 1=min
                                            date: 0=year 1=month 2=day */

/* menu */
static int menu_id;                      /* which menu page */
static int menu_sel;
enum { M_MAIN, M_RADIO, M_PRESETS, M_ALARM, M_CLOCK, M_DISPLAY };

static const uint32_t preset_khz[] = {101900, 100300};
static const char *menu_presets[] = {"101.9 FM","100.3 FM","Back"};

static const char *menu_main[]    = {"Radio","Alarm","Clock","Display","Back"};
static const char *menu_radio[]   = {"Tune","Presets","Seek Up","Seek Down",
                                     "","Diagnostics","Back"};
static const char *menu_alarm[N_ALARMS + 2];
static const char *menu_clock[]   = {"Set Time","Set Date","","","Back"};
static const char *menu_display[] = {"Contrast +","Contrast -","Back"};

static void fmt_hm(char *out, int n, uint8_t h, uint8_t m);

/* labels rebuilt on every call so toggle states are always current */
static char lbl_alarm[N_ALARMS][16];
static char lbl_snooze[20], lbl_radio_on[12], lbl_fmt[12], lbl_tz[16];

static const char **menu_items(int id, int *n)
{
    switch (id) {
    case M_RADIO:
        snprintf(lbl_radio_on, sizeof lbl_radio_on, "Radio: %s",
                 radio_on ? "ON" : "off");
        menu_radio[4] = lbl_radio_on;
        *n = 7; return menu_radio;
    case M_PRESETS: *n = 3; return menu_presets;
    case M_ALARM:
        for (int i = 0; i < N_ALARMS; i++) {
            char t[9];
            fmt_hm(t, sizeof t, alarm_h[i], alarm_m[i]);
            snprintf(lbl_alarm[i], sizeof lbl_alarm[i], "%u %s %s",
                     i + 1, t, (alarm_en_mask >> i) & 1 ? "ON" : "off");
            menu_alarm[i] = lbl_alarm[i];
        }
        snprintf(lbl_snooze, sizeof lbl_snooze,
                 menu_editing == EDIT_SNOOZE ? "Snooze: <%u> min"
                                             : "Snooze: %u min",
                 snooze_len);
        menu_alarm[N_ALARMS]     = lbl_snooze;
        menu_alarm[N_ALARMS + 1] = "Back";
        *n = N_ALARMS + 2; return menu_alarm;
    case M_CLOCK:
        snprintf(lbl_fmt, sizeof lbl_fmt, "Time: %s", fmt24 ? "24h" : "12h");
        if (tz2_off)
            snprintf(lbl_tz, sizeof lbl_tz,
                     menu_editing == EDIT_TZ2 ? "TZ2: <%+dh>" : "TZ2: %+dh",
                     tz2_off);
        else
            snprintf(lbl_tz, sizeof lbl_tz,
                     menu_editing == EDIT_TZ2 ? "TZ2: <off>" : "TZ2: off");
        menu_clock[2] = lbl_fmt;
        menu_clock[3] = lbl_tz;
        *n = 5; return menu_clock;
    case M_DISPLAY: *n = 3; return menu_display;
    default:        *n = 5; return menu_main;
    }
}

/* ---------------- audio routing ---------------- */
static void apply_audio(void)
{
    if (ringing) return;                 /* audio.c drives the amp while ringing */
    if (radio_on && !muted) {
        rda_set_volume(volume);
        rda_mute(false);
        audio_amp_enable(true);
    } else {
        rda_mute(true);
        audio_amp_enable(false);
    }
}

/* ---------------- helpers ---------------- */
static void to12(uint8_t h24, uint8_t *h12, const char **ap)
{
    *ap = (h24 < 12) ? "AM" : "PM";
    uint8_t h = h24 % 12; if (h == 0) h = 12;
    *h12 = h;
}

/* hh:mm honoring the 12/24h setting: "07:30" or "7:30PM" */
static void fmt_hm(char *out, int n, uint8_t h, uint8_t m)
{
    if (fmt24) { snprintf(out, n, "%02u:%02u", h, m); return; }
    uint8_t h12; const char *ap;
    to12(h, &h12, &ap);
    snprintf(out, n, "%u:%02u%s", h12, m, ap);
}

static void rds_clear(void)
{
    memset(rds, ' ', 8); rds[8] = '\0';
}

static void cfg_touch(void)
{
    cfg_dirty = true; cfg_at = HAL_GetTick();
}

/* play (or restart, so value changes take effect) a short tone preview */
static void sound_preview(uint8_t snd, uint8_t vol)
{
    audio_alarm_config(snd, vol);
    if (ringing) return;
    audio_alarm_stop();                  /* no-op if idle */
    audio_alarm_start();
    preview_until = HAL_GetTick() + 600;
}

static void cfg_save_now(void)
{
    /* after a seek our shadow freq is stale - ask the chip where it landed.
     * Only trust READCHAN once the tune completed (STC) and it's in-band:
     * mid-tune it reads back garbage (seen: "118.9 MHz"). */
    rda_status_t rs;
    if (radio_on && rda_get_status(&rs) && rs.stc &&
        rs.freq_khz >= 87500 && rs.freq_khz <= 108000)
        radio_khz = rs.freq_khz;

    settings_t s;
    memset(&s, 0, sizeof s);
    s.volume     = volume;
    s.radio_khz  = radio_khz;
    for (int i = 0; i < N_ALARMS; i++) {
        s.alarm_h[i] = alarm_h[i];
        s.alarm_m[i] = alarm_m[i];
    }
    s.alarm_en   = alarm_en_mask;
    for (int i = 0; i < N_ALARMS; i++) {
        s.alarm_vol[i]   = alarm_vol[i];
        s.alarm_sound[i] = alarm_snd[i];
    }
    s.snooze_len = snooze_len;
    s.contrast   = contrast;
    s.tz2        = tz2_off;
    s.flags      = (fmt24    ? SET_F_FMT24    : 0)
                 | (radio_on ? SET_F_RADIO_ON : 0);
    settings_save(&s);
}

static void seek_begin(bool up)
{
    radio_on = true; muted = false;
    apply_audio();
    rda_seek_start(up);
    rds_clear();
    cfg_touch();
}

/* for discrete on/off actions likely followed by a power pull: skip the
 * 3s debounce so the state can't be lost */
static void cfg_save_immediately(void)
{
    cfg_dirty = false;
    cfg_save_now();
}

static void cfg_load(void)
{
    settings_t s;
    if (!settings_load(&s)) return;          /* first boot - keep defaults */
    volume     = s.volume & 0x0F;
    if (s.radio_khz >= 87500 && s.radio_khz <= 108000) radio_khz = s.radio_khz;
    for (int i = 0; i < N_ALARMS; i++) {
        alarm_h[i] = s.alarm_h[i] % 24;
        alarm_m[i] = s.alarm_m[i] % 60;
    }
    alarm_en_mask = s.alarm_en & ((1u << N_ALARMS) - 1);
    for (int i = 0; i < N_ALARMS; i++) {
        alarm_vol[i] = (s.alarm_vol[i] >= 1 && s.alarm_vol[i] <= 15)
                       ? s.alarm_vol[i] : 12;
        alarm_snd[i] = s.alarm_sound[i] % AUDIO_N_SOUNDS;
    }
    snooze_len = (s.snooze_len >= 1 && s.snooze_len <= 15) ? s.snooze_len : 9;
    contrast   = s.contrast;
    tz2_off    = (s.tz2 >= -12 && s.tz2 <= 14) ? s.tz2 : 0;
    fmt24      = (s.flags & SET_F_FMT24)    != 0;
    radio_on   = (s.flags & SET_F_RADIO_ON) != 0;
}

/* index of the enabled alarm that fires soonest after `now`, or -1 */
static int next_alarm(void)
{
    int best = -1;
    uint16_t best_dt = 0xFFFF;
    uint16_t nowm = (uint16_t)(now.hour * 60 + now.min);
    for (int i = 0; i < N_ALARMS; i++) {
        if (!((alarm_en_mask >> i) & 1)) continue;
        uint16_t am = (uint16_t)(alarm_h[i] * 60 + alarm_m[i]);
        uint16_t dt = (uint16_t)((am - nowm + 1440) % 1440);
        if (dt < best_dt) { best_dt = dt; best = i; }
    }
    return best;
}

static uint8_t days_in_month(uint8_t m, uint8_t y)
{
    static const uint8_t d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (m == 2 && (y % 4) == 0) return 29;   /* valid for 2000..2099 */
    return d[m - 1];
}

/* ---------------- rendering ---------------- */
static void draw_home(void)
{
    char buf[24];
    oled_clear();

    /* status line: next alarm to fire */
    int na = next_alarm();
    if (na >= 0) {
        char t[9];
        fmt_hm(t, sizeof t, alarm_h[na], alarm_m[na]);
        snprintf(buf, sizeof buf, "AL%d %s", na + 1, t);
        oled_text(0, 0, buf, false);
    }
    if (snoozed) oled_text(78, 0, "SNZ", false);

    /* big time HH:MM */
    uint8_t hh = now.hour; const char *ap = "";
    if (!fmt24) to12(now.hour, &hh, &ap);
    if (fmt24) snprintf(buf, sizeof buf, "%02u:%02u", hh, now.min);
    else       snprintf(buf, sizeof buf, "%2u:%02u", hh, now.min);
    oled_text_big(4, 18, buf);                 /* 5 chars * 12 = 60px */
    snprintf(buf, sizeof buf, "%02u", now.sec);
    oled_text(68, 26, buf, false);
    if (!fmt24) oled_text(68, 18, ap, false);

    /* date + optional second-time-zone clock */
    snprintf(buf, sizeof buf, "20%02u-%02u-%02u", now.year, now.month, now.day);
    oled_text(4, 40, buf, false);
    if (tz2_off) {
        char t2[9];
        fmt_hm(t2, sizeof t2,
               (uint8_t)((now.hour + 24 + tz2_off) % 24), now.min);
        snprintf(buf, sizeof buf, "T2 %s", t2);
        oled_text(68, 40, buf, false);
    }

    /* radio / volume line */
    if (radio_on && !muted) {
        snprintf(buf, sizeof buf, "FM%lu.%lu %s",
                 (unsigned long)(radio_khz/1000),
                 (unsigned long)((radio_khz%1000)/100), rds);
    } else {
        snprintf(buf, sizeof buf, "%s", muted ? "MUTED" : "RADIO OFF");
    }
    oled_text(0, 56, buf, false);
    snprintf(buf, sizeof buf, "V%u", volume);
    oled_text(110, 56, buf, false);

    oled_update();
}

static void draw_menu(void)
{
    int n; const char **items = menu_items(menu_id, &n);
    oled_clear();
    oled_text(0, 0, "MENU", false);
    /* show up to 5 rows, page-scroll to keep selection visible */
    int top = menu_sel - 2; if (top < 0) top = 0;
    if (top > n - 5) top = (n > 5) ? n - 5 : 0;
    for (int i = 0; i < 5 && (top + i) < n; i++) {
        int idx = top + i;
        char line[22];
        snprintf(line, sizeof line, "%c%s", idx == menu_sel ? '>' : ' ', items[idx]);
        oled_text(0, 14 + i * 10, line, idx == menu_sel);
    }
    oled_update();
}

/* big hh:mm at (20,22) honoring 12/24h, AM/PM small at (92,ap_y) */
static void draw_big_time(uint8_t h, uint8_t m, int ap_y)
{
    char buf[8];
    uint8_t hh = h; const char *ap = "";
    if (!fmt24) to12(h, &hh, &ap);
    snprintf(buf, sizeof buf, "%2u:%02u", hh, m);
    oled_text_big(20, 22, buf);
    if (!fmt24) oled_text(92, ap_y, ap, false);
}

static void draw_edit(const char *title, uint8_t h, uint8_t m)
{
    oled_clear();
    oled_text(0, 0, title, false);
    draw_big_time(h, m, 26);
    /* underline the active field */
    int ux = (edit_field == 0) ? 20 : 56;
    oled_text(ux, 44, "^^", false);
    oled_text(0, 56, "rot=set click=next", false);
    oled_update();
}

/* the active field is drawn inverted (or underlined, for the big digits) */
static void draw_alarm_edit(void)
{
    char buf[20];
    oled_clear();
    snprintf(buf, sizeof buf, "ALARM %u", cur_alarm + 1);
    oled_text(0, 0, buf, false);
    bool en = (alarm_en_mask >> cur_alarm) & 1;
    oled_text(98, 0, en ? "ON" : "off", edit_field == 4);

    uint8_t hh = alarm_h[cur_alarm]; const char *ap = "";
    if (!fmt24) to12(alarm_h[cur_alarm], &hh, &ap);
    if (fmt24) snprintf(buf, sizeof buf, "%02u:%02u", hh, alarm_m[cur_alarm]);
    else       snprintf(buf, sizeof buf, "%2u:%02u", hh, alarm_m[cur_alarm]);
    oled_text_big(20, 12, buf);
    if (!fmt24) oled_text(92, 16, ap, false);
    if (edit_field < 2) oled_text(edit_field == 0 ? 20 : 56, 30, "^^", false);

    snprintf(buf, sizeof buf, "Sound: %s", audio_sound_name(alarm_snd[cur_alarm]));
    oled_text(0, 40, buf, edit_field == 2);
    snprintf(buf, sizeof buf, "Volume: %u", alarm_vol[cur_alarm]);
    oled_text(0, 52, buf, edit_field == 3);
    oled_update();
}

static void draw_edit_date(void)
{
    char buf[16];
    oled_clear();
    oled_text(0, 0, "SET DATE", false);
    snprintf(buf, sizeof buf, "20%02u-%02u-%02u",
             edit_t.year, edit_t.month, edit_t.day);
    oled_text_big(4, 22, buf);               /* 10 chars * 12px = 120px */
    static const int ux[3] = {28, 64, 100};  /* under YY, MM, DD */
    oled_text(ux[edit_field], 40, "^^", false);
    oled_text(0, 56, "rot=set click=next", false);
    oled_update();
}

static void draw_diag(void)
{
    rda_status_t s;
    char buf[24];
    oled_clear();
    oled_text(0, 0, "RADIO DIAG", false);
    if (!rda_get_status(&s)) {
        oled_text(0, 16, "tuner not responding", false);
        oled_update();
        return;
    }
    snprintf(buf, sizeof buf, "%lu.%lu MHz %s",
             (unsigned long)(s.freq_khz / 1000),
             (unsigned long)((s.freq_khz % 1000) / 100),
             s.stereo ? "STEREO" : "MONO");
    oled_text(0, 12, buf, false);
    snprintf(buf, sizeof buf, "RSSI %u/127", s.rssi);
    oled_text(0, 22, buf, false);
    int w = (s.rssi * OLED_W) / 127;         /* live signal bar */
    for (int y = 32; y < 38; y++)
        for (int x = 0; x < w; x++) oled_set_pixel(x, y, true);
    snprintf(buf, sizeof buf, "SIG:%c RDY:%c STC:%c",
             s.fm_true ? 'Y' : 'n', s.fm_ready ? 'Y' : 'n', s.stc ? 'Y' : 'n');
    oled_text(0, 42, buf, false);
    snprintf(buf, sizeof buf, "RDS:%c A:%u B:%u SF:%c",
             s.rds_sync ? 'Y' : 'n', s.blera, s.blerb, s.sf ? 'Y' : 'n');
    oled_text(0, 52, buf, false);
    oled_update();
}

static void draw_tune(void)
{
    char buf[24];
    oled_clear();
    oled_text(0, 0, "TUNE", false);
    snprintf(buf, sizeof buf, "%lu.%lu",
             (unsigned long)(radio_khz/1000),
             (unsigned long)((radio_khz%1000)/100));
    oled_text_big(16, 22, buf);
    oled_text(0, 48, rds, false);
    oled_text(0, 56, "rot=freq click=back", false);
    oled_update();
}

static void draw_ring(void)
{
    char buf[16];
    oled_clear();
    oled_text_big(10, 8, "ALARM");
    uint8_t hh = now.hour; const char *ap = "";
    if (!fmt24) to12(now.hour, &hh, &ap);
    snprintf(buf, sizeof buf, "%2u:%02u", hh, now.min);
    oled_text_big(28, 30, buf);
    if (!fmt24) oled_text(92, 34, ap, false);
    oled_text(0, 56, "click=snz long=off", false);
    oled_update();
}

static void render(void)
{
    switch (st) {
    case ST_HOME:     draw_home(); break;
    case ST_MENU:     draw_menu(); break;
    case ST_SET_TIME: draw_edit("SET TIME", edit_t.hour, edit_t.min); break;
    case ST_SET_DATE: draw_edit_date(); break;
    case ST_SET_ALARM:draw_alarm_edit(); break;
    case ST_TUNE:     draw_tune(); break;
    case ST_DIAG:     draw_diag(); break;
    case ST_RING:     draw_ring(); break;
    }
}

/* ---------------- menu actions ---------------- */
static void start_ring(void)
{
    if (preview_until) { preview_until = 0; audio_alarm_stop(); }
    ringing = true; snoozed = false;
    rda_mute(true);                       /* radio must not play during alarm */
    audio_alarm_config(alarm_snd[ring_alarm], alarm_vol[ring_alarm]);
    audio_alarm_start();
    st = ST_RING;
}
static void stop_ring(bool snooze)
{
    audio_alarm_stop();
    ringing = false;
    if (snooze) { snoozed = true; snooze_at = HAL_GetTick() + snooze_len*60000u; }
    else        { snoozed = false; }
    apply_audio();                        /* radio may resume (allowed while snoozed) */
    st = ST_HOME;
}

static void menu_back(void)
{
    switch (menu_id) {
    case M_MAIN:    st = ST_HOME; break;
    case M_PRESETS: menu_id = M_RADIO; menu_sel = 0; break;   /* up to Radio */
    default:        menu_id = M_MAIN; menu_sel = 0; break;
    }
}

static void menu_select(void)
{
    int n; const char **items = menu_items(menu_id, &n);
    const char *it = items[menu_sel];

    if (!strcmp(it, "Back")) { menu_back(); return; }
    if (menu_id == M_MAIN) {
        if (!strcmp(it,"Radio"))   menu_id = M_RADIO;
        if (!strcmp(it,"Alarm"))   menu_id = M_ALARM;
        if (!strcmp(it,"Clock"))   menu_id = M_CLOCK;
        if (!strcmp(it,"Display")) menu_id = M_DISPLAY;
        menu_sel = 0; return;
    }
    if (menu_id == M_RADIO) {
        if (!strcmp(it,"Tune"))    { st = ST_TUNE; return; }
        if (!strcmp(it,"Presets")) { menu_id = M_PRESETS; menu_sel = 0; return; }
        if (!strcmp(it,"Seek Up"))   { seek_begin(true);  st = ST_HOME; return; }
        if (!strcmp(it,"Seek Down")) { seek_begin(false); st = ST_HOME; return; }
        if (!strncmp(it,"Radio:",6)) {    radio_on = !radio_on; muted = false;
                                          if (radio_on) { rda_set_freq_khz(radio_khz);
                                                          rds_clear(); }
                                          apply_audio(); cfg_save_immediately();
                                          return; }
        if (!strcmp(it,"Diagnostics")) { st = ST_DIAG; return; }
    }
    if (menu_id == M_PRESETS) {
        radio_khz = preset_khz[menu_sel];
        radio_on = true; muted = false;
        rda_set_freq_khz(radio_khz);
        rds_clear();
        apply_audio();
        cfg_touch();
        st = ST_HOME;                       /* tuned - back to clock */
        return;
    }
    if (menu_id == M_ALARM) {
        if (menu_sel < N_ALARMS)  { cur_alarm = (uint8_t)menu_sel; edit_field = 0;
                                    st = ST_SET_ALARM; return; }
        if (menu_sel == N_ALARMS) { menu_editing = EDIT_SNOOZE; return; }
    }
    if (menu_id == M_CLOCK) {
        if (!strcmp(it,"Set Time")) { ds3231_get_time(&edit_t); edit_field = 0;
                                      st = ST_SET_TIME; return; }
        if (!strcmp(it,"Set Date")) { ds3231_get_time(&edit_t); edit_field = 0;
                                      if (edit_t.month < 1) edit_t.month = 1;
                                      if (edit_t.day   < 1) edit_t.day   = 1;
                                      st = ST_SET_DATE; return; }
        if (!strncmp(it,"Time:",5)) { fmt24 = !fmt24; cfg_touch(); return; }
        if (!strncmp(it,"TZ2:",4))  { menu_editing = EDIT_TZ2; return; }
    }
    if (menu_id == M_DISPLAY) {
        if (!strcmp(it,"Contrast +")) { contrast = (contrast > 0xE0) ? 0xFF : contrast + 0x20; }
        if (!strcmp(it,"Contrast -")) { contrast = (contrast < 0x20) ? 0x00 : contrast - 0x20; }
        oled_contrast(contrast);
        cfg_touch();
    }
}

/* ---------------- per-state input handling ---------------- */
static void handle_home(int rot, btn_event_t b)
{
    if (rot) {
        int v = (int)volume + rot;
        if (v < 0) v = 0;
        if (v > 15) v = 15;
        volume = (uint8_t)v;
        apply_audio();
        cfg_touch();
    }
    switch (b) {
    case BTN_CLICK:  muted = !muted; apply_audio(); break;
    case BTN_DOUBLE: menu_id = M_MAIN; menu_sel = 0; st = ST_MENU; break;
    case BTN_LONG:   if (snoozed) { snoozed = false; break; } /* kill snoozed
                                     alarm for good; radio toggle untouched */
                     radio_on = !radio_on; muted = false;
                     if (radio_on) { rda_set_freq_khz(radio_khz); rds_clear(); }
                     apply_audio(); cfg_save_immediately(); break;
    default: break;
    }
}

static void handle_menu(int rot, btn_event_t b)
{
    if (menu_editing) {                  /* rotate = value, click = accept */
        if (rot) {
            if (menu_editing == EDIT_SNOOZE) {
                int v = (int)snooze_len + rot;
                if (v < 1) v = 1;
                if (v > 15) v = 15;
                snooze_len = (uint8_t)v;
            } else if (menu_editing == EDIT_TZ2) {
                int v = (int)tz2_off + rot;    /* 0 in range = "off" */
                if (v < -12) v = -12;
                if (v > 14) v = 14;
                tz2_off = (int8_t)v;
            }
        }
        if (b == BTN_CLICK || b == BTN_LONG) { menu_editing = EDIT_NONE; cfg_touch(); }
        if (b == BTN_DOUBLE) { menu_editing = EDIT_NONE; cfg_touch(); st = ST_HOME; }
        return;
    }
    int n; menu_items(menu_id, &n);
    if (rot) {
        menu_sel = ((menu_sel + rot) % n + n) % n;   /* wrap both ways */
    }
    if (b == BTN_CLICK)  menu_select();
    if (b == BTN_LONG)   menu_back();
    if (b == BTN_DOUBLE) st = ST_HOME;
}

static void edit_adjust(uint8_t *h, uint8_t *m, int rot)
{
    if (edit_field == 0) *h = (uint8_t)((*h + rot + 24) % 24);
    else                 *m = (uint8_t)((*m + rot + 60) % 60);
}

static void handle_set_time(int rot, btn_event_t b)
{
    if (rot) edit_adjust(&edit_t.hour, &edit_t.min, rot);
    if (b == BTN_CLICK) { if (edit_field == 0) edit_field = 1;
                          else { edit_t.sec = 0; ds3231_set_time(&edit_t);
                                 st = ST_MENU; } }
    if (b == BTN_LONG) st = ST_MENU;     /* cancel */
}

static void handle_set_date(int rot, btn_event_t b)
{
    if (rot) {
        if (edit_field == 0)
            edit_t.year  = (uint8_t)((edit_t.year + rot + 100) % 100);
        else if (edit_field == 1)
            edit_t.month = (uint8_t)((edit_t.month - 1 + rot + 120) % 12 + 1);
        uint8_t dim = days_in_month(edit_t.month, edit_t.year);
        if (edit_field == 2)
            edit_t.day = (uint8_t)((edit_t.day - 1 + rot + 10 * dim) % dim + 1);
        else if (edit_t.day > dim)          /* year/month change shrank the month */
            edit_t.day = dim;
    }
    if (b == BTN_CLICK) {
        if (edit_field < 2) edit_field++;
        else {
            rtc_time_t t;                    /* keep the running time-of-day */
            if (ds3231_get_time(&t)) {
                t.day = edit_t.day; t.month = edit_t.month; t.year = edit_t.year;
                ds3231_set_time(&t);
            }
            st = ST_MENU;
        }
    }
    if (b == BTN_LONG) st = ST_MENU;         /* cancel */
}

/* fields: 0=hour 1=min 2=sound 3=volume 4=on/off */
static void handle_set_alarm(int rot, btn_event_t b)
{
    uint8_t i = cur_alarm;
    if (rot) {
        switch (edit_field) {
        case 0: case 1:
            edit_adjust(&alarm_h[i], &alarm_m[i], rot);
            break;
        case 2:
            alarm_snd[i] = (uint8_t)(((int)alarm_snd[i] + rot
                                      + 8 * AUDIO_N_SOUNDS) % AUDIO_N_SOUNDS);
            sound_preview(alarm_snd[i], alarm_vol[i]);
            break;
        case 3: {
            int v = (int)alarm_vol[i] + rot;
            if (v < 1) v = 1;
            if (v > 15) v = 15;
            alarm_vol[i] = (uint8_t)v;
            sound_preview(alarm_snd[i], alarm_vol[i]);
            break;
        }
        case 4:
            if (rot > 0) alarm_en_mask |=  (uint8_t)(1u << i);
            else         alarm_en_mask &= (uint8_t)~(1u << i);
            break;
        }
    }
    if (b == BTN_CLICK) {
        if (edit_field < 4) edit_field++;
        else { cfg_touch(); st = ST_MENU; }
    }
    if (b == BTN_LONG) { cfg_touch(); st = ST_MENU; }
}

static void handle_tune(int rot, btn_event_t b)
{
    if (rot) {
        /* North American FM grid: 200 kHz steps on odd tenths, 87.9-107.9 */
        long f = (long)radio_khz + rot * 200;
        if (f < 87900) f = 87900;
        if (f > 107900) f = 107900;
        radio_khz = (uint32_t)f;
        radio_on = true; muted = false;
        rda_set_freq_khz(radio_khz);
        rds_clear();
        apply_audio();
        cfg_touch();
    }
    if (b == BTN_CLICK || b == BTN_LONG) st = ST_MENU;
}

static void handle_diag(btn_event_t b)
{
    if (b == BTN_CLICK || b == BTN_LONG) st = ST_MENU;
}

static void handle_ring(btn_event_t b)
{
    if (b == BTN_CLICK) stop_ring(true);     /* snooze */
    if (b == BTN_LONG)  stop_ring(false);    /* dismiss */
}

/* ---------------- public API ---------------- */
void ui_init(void)
{
    cfg_load();                  /* before init so restored values apply */
    input_init();
    audio_init();
    oled_init();
    oled_contrast(contrast);
    ds3231_init();
    rda_init();
    rda_mute(true);
    ds3231_get_time(&now);
    if (radio_on)                    /* resume last station - but tune only
                                        once the tuner's crystal is stable;
                                        stay muted until then */
        resume_tune_at = HAL_GetTick() + 800;
    else
        apply_audio();
    render();
}

void ui_tick(void)
{
    static uint32_t t_input, t_sec, t_render, t_audio;
    uint32_t ms = HAL_GetTick();

    /* input poll @5ms */
    if (ms - t_input >= INPUT_TICK_MS) {
        t_input = ms;
        input_tick();
        int rot = input_get_rotation();
        btn_event_t b = input_get_button();
        if (rot || b != BTN_NONE) {
            switch (st) {
            case ST_HOME:      handle_home(rot, b); break;
            case ST_MENU:      handle_menu(rot, b); break;
            case ST_SET_TIME:  handle_set_time(rot, b); break;
            case ST_SET_DATE:  handle_set_date(rot, b); break;
            case ST_SET_ALARM: handle_set_alarm(rot, b); break;
            case ST_TUNE:      handle_tune(rot, b); break;
            case ST_DIAG:      handle_diag(b); break;
            case ST_RING:      handle_ring(b); break;
            }
        }
    }

    /* alarm beep pattern @50ms */
    if (ms - t_audio >= 50) { t_audio = ms; audio_tick(); }

    /* RDS @20Hz - groups arrive ~11/s and the chip only holds the latest,
     * so a 10 Hz poll lets some get overwritten; a faster poll shortens
     * the time to refresh all four PS pair positions, which is what the
     * boundary-locked frame assembly in rda5807.c races against */
    static uint32_t t_rds;
    if (radio_on && !muted && !rda_seek_busy() && ms - t_rds >= 50) {
        t_rds = ms;
        rda_poll_rds(rds);
    }

    /* firmware seek: step candidates, mirror the sweep onto the display */
    static uint32_t t_seek;
    static bool was_seeking;
    if (rda_seek_busy()) {
        was_seeking = true;
        if (ms - t_seek >= 30) {
            t_seek = ms;
            rda_seek_tick();
            radio_khz = rda_get_freq_khz();
        }
    } else if (was_seeking) {
        was_seeking = false;
        radio_khz = rda_get_freq_khz();   /* where the seek landed */
        cfg_touch();
    }

    /* once per second: refresh time, RDS, evaluate alarm */
    if (ms - t_sec >= 1000) {
        t_sec = ms;
        ds3231_get_time(&now);

        /* per-alarm once-per-minute tracking, so alarms can't block each
         * other: a snoozed session no longer suppresses other alarms (a new
         * alarm supersedes the pending re-ring), and two alarms in the same
         * minute ring back-to-back as each is dismissed */
        if (fired_min != now.min) { fired_min = now.min; fired_mask = 0; }
        if (!ringing) {
            for (int i = 0; i < N_ALARMS; i++) {
                if (((alarm_en_mask >> i) & 1) && !((fired_mask >> i) & 1) &&
                    now.hour == alarm_h[i] && now.min == alarm_m[i]) {
                    fired_mask |= (uint8_t)(1u << i);
                    ring_alarm = (uint8_t)i;   /* snooze re-rings this one */
                    start_ring();
                    break;
                }
            }
        }
    }

    /* deferred power-on tune (see resume_tune_at) */
    if (resume_tune_at && (int32_t)(ms - resume_tune_at) >= 0) {
        resume_tune_at = 0;
        if (radio_on) {              /* unless the user already toggled it */
            rda_set_freq_khz(radio_khz);
            apply_audio();
        }
    }

    /* snooze expiry */
    if (snoozed && (int32_t)(ms - snooze_at) >= 0) start_ring();

    /* end-of-preview for the alarm sound/volume tone */
    if (preview_until && !ringing && (int32_t)(ms - preview_until) >= 0) {
        preview_until = 0;
        audio_alarm_stop();
        apply_audio();
    }

    /* deferred settings write, 3s after the last change */
    if (cfg_dirty && ms - cfg_at >= 3000) { cfg_dirty = false; cfg_save_now(); }

    /* render @~10 fps (or immediately handled above by state change) */
    if (ms - t_render >= 100) { t_render = ms; render(); }
}

/* ---------------- software-interface hooks (USB console) ----------------
 * Run in main-loop context (console_tick), so they may touch I2C freely.
 * The 10 fps render loop picks the new state up automatically. */
void ui_cmd_radio(bool on)
{
    radio_on = on; muted = false;
    if (on) { rda_set_freq_khz(radio_khz); rds_clear(); }
    apply_audio(); cfg_save_immediately();
}

void ui_cmd_tune(uint32_t khz)
{
    radio_khz = khz; radio_on = true; muted = false;
    rda_set_freq_khz(khz); rds_clear();
    apply_audio(); cfg_touch();
}

void ui_cmd_seek(bool up)
{
    seek_begin(up);
}

void ui_cmd_vol(uint8_t v)  { volume = v & 0x0F; apply_audio(); cfg_touch(); }
void ui_cmd_mute(bool m)    { muted = m; apply_audio(); }
void ui_cmd_fmt24(bool f24) { fmt24 = f24; cfg_touch(); }

void ui_cmd_alarm(int idx, int h, int m)
{
    alarm_h[idx] = (uint8_t)h; alarm_m[idx] = (uint8_t)m;
    alarm_en_mask |= (uint8_t)(1u << idx);   /* setting a time arms it */
    cfg_touch();
}

void ui_cmd_alarm_en(int idx, bool en)
{
    if (en) alarm_en_mask |=  (uint8_t)(1u << idx);
    else    alarm_en_mask &= (uint8_t)~(1u << idx);
    cfg_touch();
}

void ui_cmd_alarm_sound(int idx, int snd)
{
    alarm_snd[idx] = (uint8_t)snd;
    sound_preview(alarm_snd[idx], alarm_vol[idx]);
    cfg_touch();
}

void ui_cmd_alarm_vol(int idx, int vol)
{
    alarm_vol[idx] = (uint8_t)vol;
    sound_preview(alarm_snd[idx], alarm_vol[idx]);
    cfg_touch();
}

void ui_cmd_snooze(uint8_t mins) { snooze_len = mins; cfg_touch(); }
void ui_cmd_tz2(int off_hours)   { tz2_off = (int8_t)off_hours; cfg_touch(); }

void ui_get_status(char *buf, int n)
{
    int len = 0;
    len += snprintf(buf + len, (size_t)(n - len),
                    "time %02u:%02u:%02u  date 20%02u-%02u-%02u  fmt %s",
                    now.hour, now.min, now.sec,
                    now.year, now.month, now.day, fmt24 ? "24h" : "12h");
    if (len < n) {
        if (tz2_off) len += snprintf(buf + len, (size_t)(n - len),
                                     "  tz2 %+dh\r\n", tz2_off);
        else         len += snprintf(buf + len, (size_t)(n - len),
                                     "  tz2 off\r\n");
    }
    if (len < n) {
        rda_status_t rs;
        int rssi = rda_get_status(&rs) ? rs.rssi : -1;
        len += snprintf(buf + len, (size_t)(n - len),
                        "radio %s  %lu.%lu MHz  vol %u%s  rssi %d  rds '%s'\r\n",
                        radio_on ? "on" : "off",
                        (unsigned long)(radio_khz / 1000),
                        (unsigned long)((radio_khz % 1000) / 100),
                        volume, muted ? " (muted)" : "", rssi, rds);
    }
    for (int i = 0; i < N_ALARMS && len < n; i++)
        len += snprintf(buf + len, (size_t)(n - len),
                        "alarm %d  %02u:%02u %s  sound %s  vol %u\r\n",
                        i + 1, alarm_h[i], alarm_m[i],
                        (alarm_en_mask >> i) & 1 ? "on " : "off",
                        audio_sound_name(alarm_snd[i]), alarm_vol[i]);
    if (len < n)
        snprintf(buf + len, (size_t)(n - len), "snooze %u min%s\r\n",
                 snooze_len,
                 ringing ? "  (RINGING)" : snoozed ? "  (snoozing)" : "");
}
