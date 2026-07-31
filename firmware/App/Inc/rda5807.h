/* rda5807.h - RDA5807M FM tuner over I2C */
#ifndef RDA5807_H
#define RDA5807_H

#include <stdint.h>
#include <stdbool.h>

void rda_init(void);                       /* power up, mono, default vol */
void rda_set_freq_khz(uint32_t khz);       /* e.g. 101100 = 101.1 MHz */
uint32_t rda_get_freq_khz(void);
void rda_set_volume(uint8_t vol);          /* 0..15 */
uint8_t rda_get_volume(void);
void rda_mute(bool mute);
bool rda_is_muted(void);

/* non-blocking seek on the North American FM grid (200 kHz, odd tenths) */
void rda_seek_start(bool up);
bool rda_seek_busy(void);
void rda_seek_tick(void);          /* poll every ~30 ms while busy */

/* Best-effort RDS Program-Service name (8 chars + NUL). Returns true if updated. */
bool rda_poll_rds(char name[9]);

/* Debug/console: copy the learned scrolling message (plus frame count) out. */
void rda_rds_msg(char *out, int outlen);

/* Debug/console: format registry row i (frame, airtime, strongest successor).
 * Returns 0 once i is past the last row. */
int rda_rds_frame(int i, char *out, int outlen);

/* Live tuner status (regs 0x0A/0x0B) for the diagnostics screen. */
typedef struct {
    uint8_t  rssi;        /* 0..127 */
    bool     fm_true;     /* chip thinks this channel is a real station */
    bool     fm_ready;
    bool     stc;         /* seek/tune complete */
    bool     sf;          /* seek failed */
    bool     stereo;      /* stereo pilot detected (output forced mono) */
    bool     rds_sync;    /* RDS decoder synchronized */
    uint8_t  blera, blerb;/* RDS block A/B error level, 0=clean 3=unusable */
    uint32_t freq_khz;    /* channel the chip is actually on (READCHAN) */
} rda_status_t;

bool rda_get_status(rda_status_t *s);

#endif /* RDA5807 */
