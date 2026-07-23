/* audio.h - alarm tone via DAC1 + TIM6 + DMA, and amp enable (AMP_SD) */
#ifndef AUDIO_H
#define AUDIO_H

#include <stdbool.h>
#include <stdint.h>

#define AUDIO_N_SOUNDS 4

void audio_init(void);
void audio_amp_enable(bool on);    /* PAM8302A SD: HIGH = on */
void audio_alarm_config(uint8_t sound, uint8_t vol);  /* sound 0..N-1, vol 1..15 */
const char *audio_sound_name(uint8_t sound);
void audio_alarm_start(void);      /* begin beeping tone */
void audio_alarm_stop(void);
void audio_tick(void);             /* call ~every 50 ms to run the beep pattern */

#endif /* AUDIO_H */
