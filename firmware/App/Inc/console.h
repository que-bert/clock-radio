/* console.h - USB CDC command console (software user interface) */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdint.h>

void console_rx(const uint8_t *buf, uint32_t len);  /* called from USB IRQ */
void console_tick(void);                            /* call from main loop */

#endif /* CONSOLE_H */
