/* oled.h - SH1106 128x64 monochrome OLED over I2C
 * NOTE: SH1106 has 132 columns internally; the visible 128 are offset by 2.
 */
#ifndef OLED_DRIVER_H
#define OLED_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#define OLED_W 128
#define OLED_H 64

void oled_init(void);
void oled_clear(void);
void oled_update(void);                 /* mark framebuffer for sending */
bool oled_pump(void);                   /* send one dirty page (~9 ms); call
                                           every main-loop pass; false = idle */

void oled_set_pixel(int x, int y, bool on);
void oled_char(int x, int y, char c, bool inverted);          /* 6x8 cell */
void oled_text(int x, int y, const char *s, bool inverted);
void oled_text_big(int x, int y, const char *s);               /* 2x scaled */
void oled_fill_row(int page, bool on);                         /* page = 8px row 0..7 */
void oled_contrast(uint8_t value);

#endif /* OLED_DRIVER_H */
