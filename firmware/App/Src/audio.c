/* audio.c - DAC alarm tone + USB-speaker PCM streaming.
 *
 * TIM6 triggers DAC1_CH1 (16 kHz alarm tones / 32 kHz USB audio). The DAC
 * runs in 12-bit mode with half-word circular DMA. Alarm tones are
 * synthesized into a 64-sample loop (freq = 250 Hz * cycles-in-buffer);
 * the alarm "beeps" by gating the amp (AMP_SD) on/off, which also avoids
 * a click on tone start.
 *
 * USB playback quality: the host is told (via a UAC feature unit) that the
 * device has hardware volume, so it always sends full-scale 16-bit samples.
 * Gain (host setting x device knob) is applied here in fixed point, then
 * the result is rounded to 12 bits with TPDF dither, which turns harmonic
 * quantization distortion into low-level hiss.
 */
#include "audio.h"
#include "bsp.h"

#define WAVE_LEN 64

enum { W_SINE, W_SQUARE, W_TRI, W_SAW };

typedef struct { const char *name; uint8_t wave; uint8_t cycles; } sound_t;

static const sound_t sounds[AUDIO_N_SOUNDS] = {
    {"Classic", W_SINE,   4},        /* 1 kHz sine */
    {"Buzzer",  W_SQUARE, 2},        /* 500 Hz square */
    {"Soft",    W_TRI,    3},        /* 750 Hz triangle */
    {"Klaxon",  W_SAW,    6},        /* 1.5 kHz sawtooth */
};

/* quarter-wave would do, but 16 entries reads easier */
static const uint8_t sine16[16] = {
    128,177,218,246,255,246,218,177,
    128, 79, 38, 10,  0, 10, 38, 79
};

static uint16_t wavebuf[WAVE_LEN];       /* 12-bit DAC samples */
static uint8_t cfg_sound = 0, cfg_vol = 12;

static bool alarm_on;
static bool beep_phase;
static uint16_t beep_ms;

const char *audio_sound_name(uint8_t sound)
{
    return sounds[sound % AUDIO_N_SOUNDS].name;
}

void audio_alarm_config(uint8_t sound, uint8_t vol)
{
    cfg_sound = sound % AUDIO_N_SOUNDS;
    cfg_vol = (vol < 1) ? 1 : (vol > 15) ? 15 : vol;
}

/* Perceptual volume: hearing is logarithmic in power, so equal loudness
 * steps need equal amplitude *ratios*, not equal increments. ~3 dB per
 * step (amplitude x1.41); the bottom entries collapse into the 8-bit
 * floor, which is unavoidable at this resolution. */
static const uint8_t vol_amp[16] = {
    0, 1, 2, 2, 3, 4, 6, 8, 11, 15, 21, 30, 42, 60, 85, 120
};

static void fill_wave(void)
{
    const sound_t *snd = &sounds[cfg_sound];
    int32_t amp = vol_amp[cfg_vol];               /* peak, of 127 */
    for (int i = 0; i < WAVE_LEN; i++) {
        uint8_t ph = (uint8_t)(i * snd->cycles * (256 / WAVE_LEN));
        int32_t s;                                /* one period, -128..127 */
        switch (snd->wave) {
        default:
        case W_SINE:   s = (int32_t)sine16[ph >> 4] - 128;              break;
        case W_SQUARE: s = (ph < 128) ? 127 : -128;                     break;
        case W_TRI:    s = (ph < 128) ? (ph * 2 - 128) : (383 - ph * 2); break;
        case W_SAW:    s = ph - 128;                                    break;
        }
        wavebuf[i] = (uint16_t)(2048 + (s * amp) / 8);   /* 12-bit centered */
    }
}

void audio_init(void)
{
    audio_amp_enable(false);
    /* CubeMX configured the DAC DMA for the old 8-bit path; the whole audio
     * path is 12-bit now, so switch the channel to half-word transfers */
    HAL_DMA_DeInit(&hdma_dac1_ch1);
    hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    HAL_DMA_Init(&hdma_dac1_ch1);
    /* park DAC at mid-scale so the line is quiet (cap blocks the DC) */
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void audio_amp_enable(bool on)
{
    HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* ---- USB speaker: PCM streamed from the host into the DAC ----
 * A circular DMA buffer free-runs at AUDIO_USB_HZ; the USB iso-OUT handler
 * writes newly received samples ~half a buffer ahead of the DMA read point
 * (a small jitter buffer). Host and MCU clocks are independent, so the write
 * point is periodically re-centered to absorb the slow drift. */
#define USB_RING 256u
static uint16_t usb_ring[USB_RING];      /* 12-bit DAC samples */
static volatile uint16_t usb_wr;
static volatile bool usb_alt;   /* host has the streaming interface selected */
static volatile bool usb_dac;   /* DAC currently driven from the USB stream */
static volatile bool usb_blocked; /* a local source (radio) owns the output */
static volatile bool usb_disabled; /* user setting: USB speaker turned off */
static volatile bool usb_dev_mute; /* device-side mute (home-screen click) */

/* silence gate: amp off after USB_SIG_HOLD_MS of near-zero signal, so an
 * idle-but-open stream doesn't hiss through the amp. Post-gain threshold of
 * 32 (s16) = 2 DAC LSBs - anything below is inaudible dither anyway. */
#define USB_SIG_THRESH   32
#define USB_SIG_HOLD_MS  300u
static volatile uint32_t usb_sig_at;     /* tick of last non-silent packet */

/* playback gain: host (USB feature unit) x device knob, recombined on
 * change so the sample loop does a single multiply */
static uint16_t host_q15 = 32767;
static uint16_t knob_q15 = 32767;
static volatile uint16_t eff_q15 = 32767;

static void usb_gain_update(void)
{
    eff_q15 = usb_dev_mute ? 0
            : (uint16_t)(((uint32_t)host_q15 * knob_q15) >> 15);
}

void audio_usb_set_host_gain(uint16_t q15)
{
    host_q15 = q15;
    usb_gain_update();
}

/* knob curve matched to the RDA5807's hardware volume (~2 dB per step,
 * measured: radio v1 ~ -28 dB): 0 dB at 15, -2 dB/step. Volume 0 continues
 * the curve (-30 dB, quietest audible) rather than muting - mute is its own
 * control (home-screen click), same split as the radio chip. The old
 * alarm-amplitude curve fell ~3 dB/step, leaving USB much quieter mid-range. */
static const uint16_t knob_db2_q15[16] = {
     1036,  1304,  1642,  2067,  2603,  3277,  4125,  5193,
     6538,  8231, 10362, 13045, 16423, 20675, 26029, 32767
};

void audio_usb_set_knob(uint8_t vol0_15)
{
    knob_q15 = knob_db2_q15[vol0_15 & 15];
    usb_gain_update();
}

void audio_usb_set_dev_mute(bool m)
{
    usb_dev_mute = m;
    usb_gain_update();
    /* zeroed samples starve the silence gate, which then closes the amp -
     * mute is audibly instant and fully hiss-free after the hold time */
}

static void audio_set_rate(uint32_t hz)
{
    uint32_t arr = 16000000u / hz - 1u;      /* TIM6 kernel clock = 16 MHz */
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr);
    __HAL_TIM_SET_COUNTER(&htim6, 0);
}

static void usb_dac_start(void)
{
    if (usb_dac || usb_blocked || usb_disabled) return;
    for (uint16_t i = 0; i < USB_RING; i++) usb_ring[i] = 2048;  /* silence */
    usb_wr = USB_RING / 2;                    /* half a buffer of latency */
    usb_sig_at = HAL_GetTick();               /* start with the gate open */
    HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
    audio_set_rate(AUDIO_USB_HZ);
    HAL_TIM_Base_Start(&htim6);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)usb_ring,
                      USB_RING, DAC_ALIGN_12B_R);
    audio_amp_enable(true);
    usb_dac = true;
}

static void usb_dac_stop(void)
{
    if (!usb_dac) return;
    usb_dac = false;
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim6);
    audio_set_rate(16000);                    /* restore the alarm-tone rate */
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    /* amp is handed back to the UI (radio) on the next apply_audio() */
}

void audio_alarm_start(void)
{
    if (alarm_on) return;
    if (usb_dac) usb_dac_stop();              /* an alarm preempts USB playback */
    alarm_on = true; beep_phase = true; beep_ms = 0;
    fill_wave();
    HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
    audio_set_rate(16000);
    HAL_TIM_Base_Start(&htim6);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)wavebuf,
                      WAVE_LEN, DAC_ALIGN_12B_R);
    audio_amp_enable(true);
}

void audio_alarm_stop(void)
{
    if (!alarm_on) return;
    alarm_on = false;
    audio_amp_enable(false);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim6);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    if (usb_alt) usb_dac_start();             /* resume USB if host still streaming */
}

void audio_usb_start(void) { usb_alt = true;  if (!alarm_on) usb_dac_start(); }
void audio_usb_stop(void)  { usb_alt = false; usb_dac_stop(); }
bool audio_usb_active(void){ return usb_dac; }

/* true while USB audio is actually audible (streaming and not silence-gated).
 * Used to postpone flash writes: a settings-page erase stalls the CPU ~40 ms,
 * which starves the 1 ms iso stream and garbles playback. */
bool audio_usb_loud(void)
{
    return usb_dac && (HAL_GetTick() - usb_sig_at < USB_SIG_HOLD_MS);
}

void audio_usb_set_blocked(bool block)
{
    if (usb_blocked == block) return;
    usb_blocked = block;
    if (block) usb_dac_stop();
    else if (usb_alt && !alarm_on) usb_dac_start();  /* start self-guards */
}

void audio_usb_set_enabled(bool en)
{
    if (usb_disabled == !en) return;
    usb_disabled = !en;
    if (usb_disabled) usb_dac_stop();
    else if (usb_alt && !alarm_on) usb_dac_start();
}

/* debug counters (see the `usbstat` console command) */
static volatile uint32_t usb_dbg_pkts, usb_dbg_dout, usb_dbg_isoinc;
static volatile uint16_t usb_dbg_lastn, usb_dbg_rd, usb_dbg_gap, usb_dbg_wr;
static volatile int16_t  usb_dbg_smp;

void audio_usb_note_dout(void)   { usb_dbg_dout++; }
void audio_usb_note_isoinc(void) { usb_dbg_isoinc++; }

void audio_usb_stats(uint32_t *pkts, uint32_t *dout, uint32_t *isoinc,
                     uint16_t *lastn, uint16_t *gap, uint16_t *wr,
                     int16_t *smp, uint8_t *flags)
{
    *pkts = usb_dbg_pkts; *dout = usb_dbg_dout; *isoinc = usb_dbg_isoinc;
    *lastn = usb_dbg_lastn; *gap = usb_dbg_gap; *wr = usb_dbg_wr; *smp = usb_dbg_smp;
    bool amp = HAL_GPIO_ReadPin(AMP_SD_GPIO_Port, AMP_SD_Pin) == GPIO_PIN_SET;
    *flags = (uint8_t)((usb_dac ? 1 : 0) | (usb_alt ? 2 : 0) |
                       (usb_blocked ? 4 : 0) | (amp ? 8 : 0));
}

/* called from the USB iso-OUT completion (interrupt context) with signed
 * 16-bit mono samples; applies gain, then rounds to the 12-bit DAC with
 * TPDF dither plus first-order noise shaping: each sample's quantization
 * error is fed back into the next, tilting the noise spectrum toward high
 * frequencies where the speaker and the ear are far less sensitive - same
 * noise power, audibly less hiss */
void audio_usb_write(const int16_t *samples, int n)
{
    static uint32_t rng = 0x2545F491u;
    static int32_t qe;                       /* error feedback, s16 units */
    usb_dbg_pkts++; usb_dbg_lastn = (uint16_t)n;
    if (n > 0) usb_dbg_smp = samples[0];
    if (!usb_dac) return;
    uint16_t rd  = (uint16_t)(USB_RING - __HAL_DMA_GET_COUNTER(hdac1.DMA_Handle1));
    uint16_t gap = (uint16_t)((usb_wr - rd + USB_RING) % USB_RING);
    usb_dbg_rd = rd; usb_dbg_gap = gap;
    if (gap < USB_RING / 4 || gap > (USB_RING * 3) / 4)
        usb_wr = (uint16_t)((rd + USB_RING / 2) % USB_RING);  /* re-center */
    uint32_t g = eff_q15;
    int32_t peak = 0;
    for (int i = 0; i < n; i++) {
        int32_t s = ((int32_t)samples[i] * (int32_t)g) >> 15;  /* gain, s16 */
        int32_t a = (s < 0) ? -s : s;
        if (a > peak) peak = a;
        int32_t x = s + 32768 + qe;                            /* + fed-back error */
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;   /* xorshift32 */
        int32_t d = (int32_t)((rng & 15u) + ((rng >> 4) & 15u)) - 15;
        int32_t v = (x + d + 8) >> 4;                          /* 12-bit round */
        if (v < 0) v = 0;
        if (v > 4095) v = 4095;
        qe = x - (v << 4);                   /* what this sample got wrong */
        if (qe > 64) qe = 64;                /* keep feedback bounded at rails */
        else if (qe < -64) qe = -64;
        usb_ring[usb_wr] = (uint16_t)v;
        usb_wr = (uint16_t)((usb_wr + 1) % USB_RING);
    }
    usb_dbg_wr = usb_wr;
    if (peak >= USB_SIG_THRESH) {            /* signal present: open the gate */
        usb_sig_at = HAL_GetTick();
        if (!alarm_on) audio_amp_enable(true);   /* instant attack */
    }
}

/* alarm beep pattern (300 ms on/off) + USB silence-gate release, @50 ms */
void audio_tick(void)
{
    if (usb_dac && !alarm_on) {
        /* close the gate once the stream has been silent for the hold time;
         * the attack (re-enable) is instant, in audio_usb_write */
        if (HAL_GetTick() - usb_sig_at >= USB_SIG_HOLD_MS)
            audio_amp_enable(false);
        return;
    }
    if (!alarm_on) return;
    beep_ms += 50;
    if (beep_ms >= 300) {
        beep_ms = 0;
        beep_phase = !beep_phase;
        audio_amp_enable(beep_phase);
    }
}
