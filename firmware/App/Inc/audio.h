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

/* --- USB speaker: PCM streamed from the host over USB Audio Class ---
 * The host's iso stream feeds the same DAC->amp path as the alarm tone, so
 * the two are mutually exclusive; an alarm preempts USB playback and USB
 * playback resumes when the alarm is dismissed. Sample rate is AUDIO_USB_HZ,
 * mono, and the class hands us signed 16-bit samples. */
#define AUDIO_USB_HZ   32000u

void audio_usb_start(void);        /* host selected the streaming interface */
void audio_usb_stop(void);         /* host stopped streaming */
void audio_usb_write(const int16_t *samples, int n);  /* from USB iso OUT */
bool audio_usb_active(void);       /* true while USB audio owns the DAC/amp */
bool audio_usb_loud(void);         /* ...and is audibly playing right now */

/* playback gain = host gain (USB feature unit) x device knob, both Q15 */
void audio_usb_set_host_gain(uint16_t q15);  /* from usbd_composite.c */
void audio_usb_set_knob(uint8_t vol0_15);    /* device volume knob 0..15 */
void audio_usb_set_dev_mute(bool m);         /* device-side mute (click) */

/* local sources own the output: while blocked (radio on), USB playback is
 * suspended and resumes automatically when unblocked (if still streaming) */
void audio_usb_set_blocked(bool block);
void audio_usb_set_enabled(bool en);         /* user setting: USB speaker off */
void audio_usb_note_dout(void);
void audio_usb_note_isoinc(void);
void audio_usb_stats(uint32_t *pkts, uint32_t *dout, uint32_t *isoinc,
                     uint16_t *lastn, uint16_t *gap, uint16_t *wr,
                     int16_t *smp, uint8_t *flags);

#endif /* AUDIO_H */
