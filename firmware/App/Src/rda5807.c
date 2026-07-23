/* rda5807.c - RDA5807M driver (sequential-access mode from reg 0x02)
 *
 * Registers are 16-bit, big-endian on the wire. Writes auto-increment from
 * 0x02. We keep a shadow of 0x02..0x07 and re-send the span we changed.
 * Band = US/Europe 87-108 MHz, spacing 100 kHz.
 *
 * NOTE: register values are conventional defaults - verify against your
 * RDA5807M datasheet and tune on hardware (esp. RDS).
 */
#include "rda5807.h"
#include "bsp.h"
#include <string.h>
#include <stdio.h>

#define BAND_BOTTOM_KHZ   87000u
#define SPACING_KHZ       100u

/* reg02 bits */
#define R02_DHIZ    0x8000
#define R02_DMUTE   0x4000   /* 1 = unmuted */
#define R02_MONO    0x2000
#define R02_SEEKUP  0x0200
#define R02_SEEK    0x0100
#define R02_RDS_EN  0x0008
#define R02_NEW     0x0004
#define R02_RESET   0x0002
#define R02_ENABLE  0x0001
/* reg03 bits */
#define R03_TUNE    0x0010

static uint16_t shadow[6];        /* index 0 = reg 0x02 ... index 5 = 0x07 */
static uint8_t  cur_vol = 6;
static uint16_t cur_chan = 0;

/* Write shadow regs 0x02..0x02+count-1 */
static void write_regs(int count)
{
    uint8_t buf[1 + 2 * 6];
    int n = 0;
    for (int i = 0; i < count; i++) {
        buf[n++] = shadow[i] >> 8;
        buf[n++] = shadow[i] & 0xFF;
    }
    HAL_I2C_Master_Transmit(&hi2c2, ADDR_RDA5807, buf, n, 50);
}

/* Write one register via the random-access address (0x11). Used where a
 * sequential write from 0x02 would re-send reg03 and cancel an in-flight
 * tune (TUNE samples as 0 -> tune aborts, ~50/50 race). */
#define ADDR_RDA5807_RND (0x11 << 1)
static void write_reg_direct(uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = {reg, val >> 8, val & 0xFF};
    HAL_I2C_Master_Transmit(&hi2c2, ADDR_RDA5807_RND, buf, 3, 50);
}

void rda_init(void)
{
    memset(shadow, 0, sizeof(shadow));

    shadow[0] = R02_ENABLE | R02_RESET;        /* soft reset */
    write_regs(1);
    HAL_Delay(50);

    shadow[0] = R02_DHIZ | R02_DMUTE | R02_MONO | R02_RDS_EN | R02_NEW | R02_ENABLE;
    shadow[1] = 0x0000;                         /* reg03: band/space=00, no tune yet */
    shadow[2] = 0x0400;                         /* reg04: DE=75us etc (default-ish) */
    shadow[3] = 0x8880 | (cur_vol & 0x0F);      /* reg05: INT_MODE, SEEKTH=8,
                                                 * LNA_PORT_SEL=LNAP (datasheet
                                                 * default 0x888B), volume */
    write_regs(4);
    HAL_Delay(50);
}

static void rds_reset(void);

void rda_set_freq_khz(uint32_t khz)
{
    rds_reset();                  /* don't mix old station's text with new */
    if (khz < BAND_BOTTOM_KHZ) khz = BAND_BOTTOM_KHZ;
    cur_chan = (uint16_t)((khz - BAND_BOTTOM_KHZ) / SPACING_KHZ);
    /* reg03: CHAN[15:6] | TUNE | BAND[3:2]=00 | SPACE[1:0]=00 */
    shadow[1] = (cur_chan << 6) | R03_TUNE;
    write_regs(2);
    /* TUNE is self-clearing in hardware; drop it from the shadow so later
     * writes (e.g. volume) don't re-trigger a tune. */
    shadow[1] &= ~R03_TUNE;
}

uint32_t rda_get_freq_khz(void)
{
    return BAND_BOTTOM_KHZ + (uint32_t)cur_chan * SPACING_KHZ;
}

void rda_set_volume(uint8_t vol)
{
    cur_vol = vol & 0x0F;
    shadow[3] = (shadow[3] & ~0x000F) | cur_vol;
    write_reg_direct(0x05, shadow[3]);
}
uint8_t rda_get_volume(void) { return cur_vol; }

void rda_mute(bool mute)
{
    if (mute) shadow[0] &= ~R02_DMUTE; else shadow[0] |= R02_DMUTE;
    write_regs(1);
}
bool rda_is_muted(void) { return (shadow[0] & R02_DMUTE) == 0; }

/* ---- firmware-driven seek on the North American grid ----
 * The chip's own seek walks its raw 100 kHz grid (or a 200 kHz grid
 * aligned to even tenths), so it can stop on channels that don't exist
 * in North America. Instead we step the NA grid ourselves - 87.9 to
 * 107.9 in 200 kHz - tune each candidate, and stop when the chip flags
 * a genuine station. Non-blocking: rda_seek_tick() is polled by the UI. */
#define NA_BOTTOM_KHZ 87900u
#define NA_CHANNELS   101u          /* 87.9 .. 107.9 */
#define SEEK_MIN_RSSI 20u           /* reject adjacent-channel splatter */

static bool    seek_on;
static bool    seek_dir_up;
static uint8_t seek_n;              /* NA grid index 0..100 */
static uint8_t seek_steps;          /* to give up after one full lap */
static uint8_t seek_settle;         /* ticks seen with STC set */

static void seek_advance(void)
{
    if (seek_dir_up) seek_n = (uint8_t)((seek_n >= NA_CHANNELS - 1) ? 0 : seek_n + 1);
    else             seek_n = (uint8_t)((seek_n == 0) ? NA_CHANNELS - 1 : seek_n - 1);
    seek_steps++;
    seek_settle = 0;
    rda_set_freq_khz(NA_BOTTOM_KHZ + (uint32_t)seek_n * 200u);
}

void rda_seek_start(bool up)
{
    long f = (long)rda_get_freq_khz() - (long)NA_BOTTOM_KHZ;
    int n = (int)((f + 100) / 200);           /* snap to nearest NA channel */
    if (n < 0) n = 0;
    if (n > (int)NA_CHANNELS - 1) n = NA_CHANNELS - 1;
    seek_n = (uint8_t)n;
    seek_dir_up = up;
    seek_steps = 0;
    seek_on = true;
    seek_advance();                           /* tune the first candidate */
}

bool rda_seek_busy(void) { return seek_on; }

void rda_seek_tick(void)
{
    if (!seek_on) return;
    rda_status_t s;
    if (!rda_get_status(&s) || !s.stc) return;   /* candidate still tuning */
    if (++seek_settle < 2) return;               /* FM_TRUE lags STC - give
                                                    it one extra tick */
    if ((s.fm_true && s.rssi >= SEEK_MIN_RSSI) ||
        seek_steps >= NA_CHANNELS) {             /* found, or full lap: stop */
        seek_on = false;
        return;
    }
    seek_advance();
}

/* ---- RDS PS-field message builder (group 0) ----
 * Many North American stations misuse the PS field as a scrolling banner,
 * cycling padded 8-char word-chunks every few seconds ("  JACK  ",
 * " 103.1  ", "  80s,  ", ...). This code learns the cycle and marquees
 * it across the display, so the text reads exactly what the station
 * means, e.g. "JACK 103.1 80s, 90s, more!".
 *
 * Evidence model. Reception gaps make the stream of assembled frames
 * skip cycle members at random, so catch order alone is not reliable.
 *  - seen_cnt[i] counts dwells: one sighting per continuous stretch of
 *    frame i, no matter how many times it re-assembles meanwhile. Junk
 *    (a chimera caught at a boundary, unflagged block-D corruption)
 *    flashes once and scores 1; a frame needs >=2 to be displayed.
 *    Per-airtime counting was tried and let chimeras outscore rare
 *    real frames.
 *  - tcount[a][b] counts observed a->b dwell changes. A dwell that ran
 *    ~1.5x longer than the learned dwell EMA probably hid a missed
 *    frame, so its transition scores 1 against 2 for a normal dwell;
 *    true adjacency then outweighs skip edges ~4:1.
 *  - both decay by half every minute so old content (a previous song)
 *    fades out and the cycle is re-learned.
 *
 * Ordering. Edges are accepted globally, strongest first, each frame
 * taking at most one successor and one predecessor (a greedy walk from
 * an anchor was tried first: one skip-edge made it jump past frames it
 * then could never revisit, orphaning the rest of the cycle). Closing a
 * loop is only allowed once it spans every displayable frame. The
 * resulting chains are emitted starting at the anchor - the most-seen
 * frame, sticky so the rotation doesn't churn - then remaining chains
 * in first-seen order, so every confirmed frame shows even while the
 * order evidence is still thin. Until anything is confirmed, the single
 * best frame shows as-is (a static PS never cycles at all, and after a
 * retune this puts a name up in about a second).
 *
 * Frame assembly is boundary-locked: a station replaces the whole 8-char
 * PS atomically, so any pair position whose characters change marks a
 * frame boundary, and a window built only from pairs that arrived at or
 * after the most recent boundary cannot mix two frames. Frames with
 * non-printable characters are dropped (block D carries no error flags
 * on this chip). */
#define RDS_MAX_FRAMES 12      /* distinct frames tracked */
#define RDS_MSG_MAX    128     /* joined message capacity */
#define SCROLL_STEP    250u    /* ms per 1-char marquee step */
#define TRANS_DECAY_MS 60000u  /* halve evidence this often */
#define RDS_MIN_SEEN   2       /* airtime bumps before a frame may display */

static char    reg[RDS_MAX_FRAMES][8];      /* frame registry */
static uint8_t nreg;
static uint16_t seen_cnt[RDS_MAX_FRAMES];   /* dwells seen per frame */
static uint8_t tcount[RDS_MAX_FRAMES][RDS_MAX_FRAMES]; /* A->B counts */
static uint32_t last_decay;
static char    prevf[8];                    /* currently-airing frame */
static bool    have_prev;
static uint32_t dwell_start;                /* when the current dwell began */
static uint32_t dwell_ema;                  /* learned dwell length, ms */
static char    anchor_ps[8];                /* sticky rotation start */
static bool    have_anchor;
static char    msg[RDS_MSG_MAX];            /* joined display message */
static uint8_t msg_len;
static uint8_t scroll_off;                  /* marquee offset */
static uint32_t scroll_ms;                  /* last step tick */

static char    pr[4][2];        /* latest pair per position */
static bool    phave[4];        /* position seen since reset */
static bool    pseen[4];        /* position received since the wave began */

static void rds_reset(void)
{
    nreg = 0; have_prev = false; have_anchor = false;
    msg_len = 0; scroll_off = 0;
    scroll_ms = HAL_GetTick();
    last_decay = scroll_ms;
    dwell_start = 0; dwell_ema = 0;
    memset(seen_cnt, 0, sizeof seen_cnt);
    memset(tcount, 0, sizeof tcount);
    memset(pr, 0, sizeof pr);
    memset(phave, 0, sizeof phave);
    memset(pseen, 0, sizeof pseen);
}

static int reg_find(const char *f)
{
    for (int i = 0; i < nreg; i++)
        if (memcmp(f, reg[i], 8) == 0) return i;
    return -1;
}

/* evict the least-seen frame (reception junk dies here) */
static void reg_evict_min(void)
{
    int m = 0;
    for (int i = 1; i < nreg; i++)
        if (seen_cnt[i] < seen_cnt[m]) m = i;
    nreg--;
    memmove(reg + m, reg + m + 1, (nreg - m) * 8);
    memmove(seen_cnt + m, seen_cnt + m + 1, (nreg - m) * sizeof seen_cnt[0]);
    memmove(tcount + m, tcount + m + 1, (nreg - m) * sizeof tcount[0]);
    for (int i = 0; i < nreg; i++) {
        memmove(&tcount[i][m], &tcount[i][m + 1], nreg - m);
        tcount[i][nreg] = 0;
    }
}

static int fr_overlap(const char *a, int la, const char *b, int lb)
{
    int max = la < lb ? la : lb;
    for (int k = max; k > 0; k--)
        if (memcmp(a + la - k, b, k) == 0) return k;
    return 0;
}

static void msg_append(const char *s, int l)
{
    if (!l) return;
    if (msg_len == 0) {
        if (l > RDS_MSG_MAX) l = RDS_MSG_MAX;
        memcpy(msg, s, l); msg_len = (uint8_t)l;
    } else {
        int o = fr_overlap(msg, msg_len, s, l);
        if (o >= 3 && msg_len + l - o <= RDS_MSG_MAX) {
            memcpy(msg + msg_len, s + o, l - o);
            msg_len = (uint8_t)(msg_len + l - o);
        } else if (msg_len + 1 + l <= RDS_MSG_MAX) {
            msg[msg_len++] = ' ';
            memcpy(msg + msg_len, s, l);
            msg_len = (uint8_t)(msg_len + l);
        }
    }
}

static void emit_frame(int i)
{
    const char *s = reg[i];
    int l = 8;
    while (l && s[0] == ' ') { s++; l--; }
    while (l && s[l - 1] == ' ') l--;
    msg_append(s, l);
}

/* accept edges globally, strongest first; emit the chains from the anchor */
static void order_rebuild(void)
{
    int8_t succ[RDS_MAX_FRAMES], pred[RDS_MAX_FRAMES];
    bool   ok[RDS_MAX_FRAMES];
    int nok = 0;
    msg_len = 0;
    for (int i = 0; i < nreg; i++) {
        succ[i] = pred[i] = -1;
        ok[i] = seen_cnt[i] >= RDS_MIN_SEEN;
    }
    /* chimera filter: a boundary caught with one stale-but-matching pair
     * assembles a frame that shares exactly 3 of 4 pairs with the real
     * one - drop any frame differing from a >=2x-more-seen frame in a
     * single pair */
    for (int i = 0; i < nreg; i++) {
        if (!ok[i]) continue;
        for (int j = 0; j < nreg && ok[i]; j++) {
            if (j == i || seen_cnt[j] < 2 * seen_cnt[i]) continue;
            int shared = 0;
            for (int p = 0; p < 4; p++)
                if (memcmp(reg[i] + 2 * p, reg[j] + 2 * p, 2) == 0) shared++;
            if (shared == 3) ok[i] = false;
        }
        if (ok[i]) nok++;
    }
    if (!nok) {              /* nothing confirmed yet: best effort, so a
                              * station name shows right after tuning */
        if (nreg) {
            int best = 0;
            for (int i = 1; i < nreg; i++)
                if (seen_cnt[i] > seen_cnt[best]) best = i;
            emit_frame(best);
        }
        msg[msg_len] = '\0';
        return;
    }

    for (;;) {
        int bi = -1, bj = -1, bc = 0;
        for (int i = 0; i < nreg; i++) {
            if (!ok[i] || succ[i] >= 0) continue;
            for (int j = 0; j < nreg; j++) {
                if (j == i || !ok[j] || pred[j] >= 0) continue;
                if (tcount[i][j] <= bc) continue;
                /* i->j may close a loop only if it spans every ok frame */
                int k = j, hops = 1;
                while (k != i && succ[k] >= 0) { k = succ[k]; hops++; }
                if (k == i && hops < nok) continue;
                bi = i; bj = j; bc = tcount[i][j];
            }
        }
        if (bi < 0) break;
        succ[bi] = (int8_t)bj; pred[bj] = (int8_t)bi;
    }

    int anchor = -1;
    for (int i = 0; i < nreg; i++)
        if (ok[i] && (anchor < 0 || seen_cnt[i] > seen_cnt[anchor])) anchor = i;
    if (have_anchor) {          /* keep the old rotation while it's near-top */
        int a = reg_find(anchor_ps);
        if (a >= 0 && ok[a] && seen_cnt[a] * 4 >= seen_cnt[anchor] * 3)
            anchor = a;
    }
    memcpy(anchor_ps, reg[anchor], 8); have_anchor = true;

    uint16_t done = 0;
    int cur = anchor;
    while (cur >= 0 && !(done & (1u << cur))) {
        done |= (uint16_t)(1u << cur);
        emit_frame(cur);
        cur = succ[cur];
    }
    for (;;) {   /* leftover chains: heads first, in first-seen order */
        int head = -1;
        for (int i = 0; i < nreg && head < 0; i++)
            if (ok[i] && !(done & (1u << i)) &&
                (pred[i] < 0 || (done & (1u << pred[i])))) head = i;
        if (head < 0) break;
        cur = head;
        while (cur >= 0 && !(done & (1u << cur))) {
            done |= (uint16_t)(1u << cur);
            emit_frame(cur);
            cur = succ[cur];
        }
    }
    msg[msg_len] = '\0';
}

/* called on every assembled frame - several times per second while one
 * frame keeps airing, so sightings and transitions are per-dwell here */
static void frames_offer(const char *f)
{
    uint32_t now = HAL_GetTick();
    if (now - last_decay >= TRANS_DECAY_MS) {   /* fade old content out */
        last_decay = now;
        for (int i = 0; i < RDS_MAX_FRAMES; i++) {
            seen_cnt[i] >>= 1;
            for (int j = 0; j < RDS_MAX_FRAMES; j++)
                tcount[i][j] >>= 1;
        }
    }
    int idx = reg_find(f);
    if (idx < 0) {
        if (nreg >= RDS_MAX_FRAMES) reg_evict_min();
        idx = nreg++;
        memcpy(reg[idx], f, 8);
        seen_cnt[idx] = 0;
        for (int k = 0; k < RDS_MAX_FRAMES; k++) {
            tcount[idx][k] = 0;
            tcount[k][idx] = 0;
        }
    }

    if (have_prev && memcmp(prevf, f, 8) == 0) return;  /* same dwell */

    /* one sighting per dwell - a chimera's single flash then scores like
     * one dwell, not like seconds of airtime */
    if (seen_cnt[idx] < 0xFFFF) seen_cnt[idx]++;
    if (have_prev) {
        uint32_t gap = now - dwell_start;
        int p = reg_find(prevf);
        if (p >= 0 && p != idx) {
            /* an overlong dwell probably hid a missed frame - count
             * that transition, but weakly */
            uint8_t w = (dwell_ema && gap > dwell_ema + dwell_ema / 2)
                        ? 1 : 2;
            tcount[p][idx] = (uint8_t)(tcount[p][idx] > 250u - w
                                       ? 250 : tcount[p][idx] + w);
        }
        if (!dwell_ema)
            dwell_ema = gap;
        else if (gap <= dwell_ema + dwell_ema / 2)
            dwell_ema += (uint32_t)(((int32_t)gap - (int32_t)dwell_ema) / 4);
    }
    dwell_start = now;
    memcpy(prevf, f, 8); have_prev = true;
    order_rebuild();
}

/* fill name[9] with the current 8-char window of the message marquee */
static void scroll_view(char name[9])
{
    uint32_t now = HAL_GetTick();
    if (msg_len > 8) {
        uint16_t loop = (uint16_t)msg_len + 2;  /* 2-space separator */
        if (now - scroll_ms >= SCROLL_STEP) {
            scroll_off = (uint8_t)((scroll_off +
                                    (now - scroll_ms) / SCROLL_STEP) % loop);
            scroll_ms = now;
        }
        for (int i = 0; i < 8; i++) {
            uint16_t idx = (uint16_t)((scroll_off + i) % loop);
            name[i] = idx < msg_len ? msg[idx] : ' ';
        }
    } else {
        for (int i = 0; i < 8; i++)
            name[i] = i < msg_len ? msg[i] : ' ';
    }
    name[8] = '\0';
}

/* debug/console: copy the learned message (and meta) out */
void rda_rds_msg(char *out, int outlen)
{
    int n = snprintf(out, outlen, "%s  [%u frames, dwell ~%lums]",
                     msg_len ? msg : "", nreg, (unsigned long)dwell_ema);
    (void)n;
}

/* debug/console: one registry row per call; returns 0 past the end */
int rda_rds_frame(int i, char *out, int outlen)
{
    if (i < 0 || i >= nreg) return 0;
    int bs = -1;
    for (int j = 0; j < nreg; j++)
        if (j != i && tcount[i][j] > (bs < 0 ? 0 : tcount[i][bs])) bs = j;
    snprintf(out, outlen, "%2d '%.8s' seen=%-3u next=%d(%u)",
             i, reg[i], seen_cnt[i], bs, bs >= 0 ? tcount[i][bs] : 0);
    return 1;
}

bool rda_poll_rds(char name[9])
{
    uint8_t r[12];               /* sequential read starts at reg 0x0A:
                                  *   r[0..1]=0x0A status   r[2..3]=0x0B
                                  *   r[4..5]=0x0C block A   r[6..7]=0x0D block B
                                  *   r[8..9]=0x0E block C   r[10..11]=0x0F block D */
    if (HAL_I2C_Master_Receive(&hi2c2, ADDR_RDA5807, r, 12, 50) != HAL_OK)
        return false;

    uint16_t reg0a = (r[0] << 8) | r[1];
    uint16_t reg0b = (r[2] << 8) | r[3];
    if (!(reg0a & 0x8000)) { scroll_view(name); return false; }   /* no new group */
    /* BLERA/BLERB: 0 = clean, 1 = 1-2 errors corrected by the decoder.
     * Accept level 1 (the correction is reliable, and on fringe stations
     * most frames arrive at this level); drop 2/3 as unreliable. */
    uint8_t blera = (reg0b >> 2) & 3, blerb = reg0b & 3;
    if (blera > 1 || blerb > 1) { scroll_view(name); return false; }

    uint16_t b = (r[6] << 8) | r[7];              /* block B (reg 0x0D) */
    uint16_t d = (r[10] << 8) | r[11];            /* block D (reg 0x0F) */
    if (((b >> 12) & 0x0F) != 0) { scroll_view(name); return false; } /* only group 0 = PS */
    uint8_t pi  = b & 0x03;                       /* pair index 0..3 */
    char c0 = (char)(d >> 8), c1 = (char)(d & 0xFF);

    /* A station replaces the whole 8-char PS atomically, so a content
     * change at a position already received this wave means the frame
     * was swapped under us: everything held so far is the old frame -
     * start a new wave. A change at a position NOT yet received this
     * wave is that pair's own (possibly loss-delayed) update, so it
     * simply joins the wave. This is deliberately not time-based: a
     * time-window version both split waves (a straggler past the window
     * looked like a new wave) and then merged the next real boundary
     * into the remains, assembling chars from two frames. */
    if (phave[pi] && (pr[pi][0] != c0 || pr[pi][1] != c1) && pseen[pi])
        memset(pseen, 0, sizeof pseen);
    pr[pi][0] = c0; pr[pi][1] = c1;
    phave[pi] = true;
    pseen[pi] = true;

    /* assemble once every position has been received in this wave; a
     * pair from before the wave is old-frame and never used, so two
     * frames cannot mix */
    bool full = true;
    for (int i = 0; i < 4; i++)
        if (!pseen[i]) { full = false; break; }
    if (full) {
        char w[8], sp = 0;
        bool printable = true;
        for (int i = 0; i < 4; i++) { w[2 * i] = pr[i][0]; w[2 * i + 1] = pr[i][1]; }
        for (int i = 0; i < 8; i++) {
            sp |= (char)(w[i] ^ ' ');
            if (w[i] < 0x20 || w[i] >= 0x7F) printable = false;
        }
        /* block D carries no error flags on this chip, so corrupted
         * characters arrive unflagged: only printable, non-blank frames
         * enter the message builder */
#ifdef RDS_HOST_TRACE
        printf("t=%6lu asm '%.8s'%s\n", (unsigned long)HAL_GetTick(), w,
               (printable && sp) ? "" : " (dropped)");
#endif
        if (printable && sp) frames_offer(w);
    }

    scroll_view(name);
    return true;
}

bool rda_get_status(rda_status_t *s)
{
    uint8_t r[4];                /* regs 0x0A, 0x0B */
    if (HAL_I2C_Master_Receive(&hi2c2, ADDR_RDA5807, r, 4, 50) != HAL_OK)
        return false;
    uint16_t a = (r[0] << 8) | r[1];
    uint16_t b = (r[2] << 8) | r[3];
    s->stc      = (a & 0x4000) != 0;
    s->sf       = (a & 0x2000) != 0;
    s->rds_sync = (a & 0x1000) != 0;
    s->stereo   = (a & 0x0400) != 0;
    s->freq_khz = BAND_BOTTOM_KHZ + (uint32_t)(a & 0x03FF) * SPACING_KHZ;
    s->rssi     = r[2] >> 1;                      /* 0x0B[15:9] */
    s->fm_true  = (b & 0x0100) != 0;
    s->fm_ready = (b & 0x0080) != 0;
    s->blera    = (b >> 2) & 3;
    s->blerb    = b & 3;
    return true;
}
