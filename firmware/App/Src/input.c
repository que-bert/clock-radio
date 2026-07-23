/* input.c - encoder via TIM2 counts, button via debounced state machine */
#include "input.h"
#include "bsp.h"

#define DETENT_COUNTS  4           /* TI12 mode: 4 counts per detent */
#define DEBOUNCE_MS    20
#define LONG_MS        600
#define DBLCLICK_MS    450

/* ---- encoder ---- */
static uint32_t last_cnt;
static int32_t  accum;             /* fractional detents */
static int      rot_pending;

/* ---- button (active-low on PA2, internal pull-up) ----
 * All timing uses HAL_GetTick() so it stays correct even when the main
 * loop is blocked (e.g. inside an OLED I2C transfer) and ticks arrive late. */
static uint8_t  last_raw, level, pressed;
static uint8_t  click_pending, long_sent;
static uint32_t raw_since;         /* last raw edge */
static uint32_t down_at;           /* when the press began */
static uint32_t click_at;          /* first tap, waiting out the double-click window */
static btn_event_t evt_pending;

void input_init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    last_cnt = __HAL_TIM_GET_COUNTER(&htim2);
}

void input_tick(void)
{
    /* ----- encoder ----- */
    uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t delta = -(int32_t)(cnt - last_cnt);  /* wraps correctly; negated to
                                                    match this encoder's A/B order */
    last_cnt = cnt;
    accum += delta;
    while (accum >= DETENT_COUNTS)  { rot_pending++; accum -= DETENT_COUNTS; }
    while (accum <= -DETENT_COUNTS) { rot_pending--; accum += DETENT_COUNTS; }

    /* ----- button ----- */
    uint32_t ms = HAL_GetTick();
    uint8_t raw = (HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin)
                   == GPIO_PIN_RESET);
    if (raw != last_raw) { last_raw = raw; raw_since = ms; }
    else if (ms - raw_since >= DEBOUNCE_MS) level = raw;

    if (level && !pressed) {                  /* just pressed */
        pressed = 1; down_at = ms; long_sent = 0;
    } else if (level && pressed) {            /* held */
        if (!long_sent && ms - down_at >= LONG_MS) {
            long_sent = 1; evt_pending = BTN_LONG; click_pending = 0;
        }
    } else if (!level && pressed) {           /* released */
        pressed = 0;
        if (!long_sent) {                     /* a tap */
            if (click_pending) { evt_pending = BTN_DOUBLE; click_pending = 0; }
            else { click_pending = 1; click_at = ms; }
        }
    }
    if (click_pending && ms - click_at >= DBLCLICK_MS) {
        evt_pending = BTN_CLICK; click_pending = 0;
    }
}

int input_get_rotation(void)
{
    int r = rot_pending; rot_pending = 0; return r;
}

btn_event_t input_get_button(void)
{
    btn_event_t e = evt_pending; evt_pending = BTN_NONE; return e;
}
