/* audio.c - DAC alarm tone.
 *
 * TIM6 triggers DAC1_CH1 at 16 kHz (set in CubeMX). A 64-sample waveform
 * buffer played in circular DMA gives freq = 250 Hz * cycles-in-buffer.
 * Waveform, frequency and volume (amplitude scaling - the DAC itself has
 * no volume) are synthesized into the buffer when the alarm starts. The
 * alarm "beeps" by gating the amp (AMP_SD) on/off, which also avoids a
 * click on tone start.
 *
 * The structure (start/stop a streamed buffer) is the hook for later
 * replacing the buffer with a custom sample streamed from flash.
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

static uint8_t wavebuf[WAVE_LEN];
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
        wavebuf[i] = (uint8_t)(128 + (s * amp) / 128);
    }
}

void audio_init(void)
{
    audio_amp_enable(false);
    /* park DAC at mid-scale so the line is quiet (cap blocks the DC) */
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 128);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

void audio_amp_enable(bool on)
{
    HAL_GPIO_WritePin(AMP_SD_GPIO_Port, AMP_SD_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void audio_alarm_start(void)
{
    if (alarm_on) return;
    alarm_on = true; beep_phase = true; beep_ms = 0;
    fill_wave();
    HAL_DAC_Stop(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Start(&htim6);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)wavebuf,
                      WAVE_LEN, DAC_ALIGN_8B_R);
    audio_amp_enable(true);
}

void audio_alarm_stop(void)
{
    if (!alarm_on) return;
    alarm_on = false;
    audio_amp_enable(false);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Stop(&htim6);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R, 128);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

/* beep pattern: 300 ms on, 300 ms off, by gating the amp */
void audio_tick(void)
{
    if (!alarm_on) return;
    beep_ms += 50;
    if (beep_ms >= 300) {
        beep_ms = 0;
        beep_phase = !beep_phase;
        audio_amp_enable(beep_phase);
    }
}
