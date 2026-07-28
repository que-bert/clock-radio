/* input.c - encoder via TIM2 counts, button via EXTI edge capture.
 *
 * The button is interrupt-driven: press/release edges are timestamped in the
 * EXTI ISR, so no press can be missed even when the main loop is blocked for
 * tens of ms (OLED I2C page writes). input_tick() only classifies the
 * captured edges into click / double / long events.
 */
#include "input.h"
#include "bsp.h"

#define DETENT_COUNTS  4           /* TI12 mode: 4 counts per detent */
#define EDGE_GUARD_MS  10          /* ignore edges closer than this (bounce) */
#define MIN_TAP_MS     30          /* shorter presses are bounce, not taps */
#define LONG_MS        600
#define DBLCLICK_MS    450

/* ---- encoder ---- */
static uint32_t last_cnt;
static int32_t  accum;             /* fractional detents */
static int      rot_pending;

/* ---- button (active-low on PA2, EXTI2) ----
 * ISR-side capture state; classification state lives in input_tick. */
static volatile uint8_t  b_pressed;      /* debounced held state */
static volatile uint32_t b_down_at;      /* when the press began */
static volatile uint32_t b_last_edge;    /* last edge of either polarity */
static volatile uint8_t  b_taps;         /* completed short presses to consume */
static volatile uint8_t  b_presses;      /* press sequence number (long re-arm) */

static uint8_t  click_pending, long_sent, last_presses;
static uint32_t click_at;          /* first tap, waiting out the double-click window */
static btn_event_t evt_pending;

/* debug: raw edge counter, to spot electrical noise on the button line */
static volatile uint32_t dbg_edges;
uint32_t input_dbg_edges(void) { return dbg_edges; }

static void btn_edge(bool press)
{
    uint32_t ms = HAL_GetTick();
    dbg_edges++;
    if (press) {
        if (!b_pressed && ms - b_last_edge >= EDGE_GUARD_MS) {
            b_pressed = 1; b_down_at = ms; b_presses++;
        }
    } else if (b_pressed) {
        uint32_t dur = ms - b_down_at;
        if (dur >= MIN_TAP_MS) {             /* real release */
            b_pressed = 0;
            if (dur < LONG_MS) b_taps++;     /* long is reported by the poll */
        }                                    /* else: bounce - stay pressed */
    }
    b_last_edge = ms;
}

void EXTI2_3_IRQHandler(void)
{
    if (EXTI->FPR1 & ENC_BUTTON_Pin) {       /* falling = press (active low) */
        EXTI->FPR1 = ENC_BUTTON_Pin;
        btn_edge(true);
    }
    if (EXTI->RPR1 & ENC_BUTTON_Pin) {       /* rising = release */
        EXTI->RPR1 = ENC_BUTTON_Pin;
        btn_edge(false);
    }
}

void input_init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    last_cnt = __HAL_TIM_GET_COUNTER(&htim2);

    /* switch the button pin from plain input to EXTI on both edges */
    GPIO_InitTypeDef g = {0};
    g.Pin  = ENC_BUTTON_Pin;
    g.Mode = GPIO_MODE_IT_RISING_FALLING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENC_BUTTON_GPIO_Port, &g);
    HAL_NVIC_SetPriority(EXTI2_3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
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

    /* ----- button: classify the ISR-captured edges ----- */
    uint32_t ms = HAL_GetTick();

    /* safety net: if the ISR missed a release (e.g. a <MIN_TAP_MS press whose
     * release was discarded as bounce), resync from the actual pin level.
     * Duration is measured to the last edge, not to now - otherwise a noise
     * glitch (press+release both discarded) would inflate into a phantom tap */
    bool raw = HAL_GPIO_ReadPin(ENC_BUTTON_GPIO_Port, ENC_BUTTON_Pin)
               == GPIO_PIN_RESET;
    if (b_pressed && !raw && ms - b_last_edge >= EDGE_GUARD_MS) {
        uint32_t dur = b_last_edge - b_down_at;
        b_pressed = 0;
        if (dur >= MIN_TAP_MS && dur < LONG_MS) b_taps++;
    }

    if (b_presses != last_presses) {         /* new press: re-arm long */
        last_presses = b_presses;
        long_sent = 0;
    }
    if (b_pressed && !long_sent && ms - b_down_at >= LONG_MS) {
        long_sent = 1; evt_pending = BTN_LONG; click_pending = 0;
    }

    uint8_t taps;
    __disable_irq();
    taps = b_taps; b_taps = 0;
    __enable_irq();
    while (taps--) {
        if (long_sent) continue;             /* release of a long press */
        if (click_pending) { evt_pending = BTN_DOUBLE; click_pending = 0; }
        else { click_pending = 1; click_at = ms; }
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
