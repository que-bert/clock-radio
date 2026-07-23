/* input.h - rotary encoder (TIM2) + push-button state machine */
#ifndef INPUT_H
#define INPUT_H

#include <stdint.h>

typedef enum {
    BTN_NONE = 0,
    BTN_CLICK,      /* short single press */
    BTN_DOUBLE,     /* two quick presses */
    BTN_LONG        /* held past threshold */
} btn_event_t;

void input_init(void);
void input_tick(void);                 /* call every INPUT_TICK_MS */
int  input_get_rotation(void);         /* signed detents since last call */
btn_event_t input_get_button(void);    /* one event per call (or BTN_NONE) */

#define INPUT_TICK_MS 5

#endif /* INPUT_H */
