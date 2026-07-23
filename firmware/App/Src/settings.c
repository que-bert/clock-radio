/* settings.c - persistent settings in the last flash page (2KB @ 0x0807F800)
 *
 * Wear leveling: the page is used as an append-only log of 16-byte slots.
 * The newest valid slot wins on load. A save writes the next free slot;
 * only when all 128 are used is the page erased (so ~128 saves per erase
 * cycle -> >1M saves over the 10k-cycle flash endurance). The firmware
 * image is ~40KB, nowhere near this page, and neither dfu-util nor the
 * OpenOCD `program` command erase pages they don't write, so settings
 * survive reflashing.
 */
#include "settings.h"
#include "bsp.h"
#include <string.h>

#define CFG_PAGE_ADDR  0x0807F800u          /* bank 2, page 127 (last 2KB) */
#define CFG_SLOT_SIZE  32u
#define CFG_SLOTS      (FLASH_PAGE_SIZE / CFG_SLOT_SIZE)
#define CFG_MAGIC      0xC10Cu
#define CFG_VERSION    4u        /* v4: per-alarm volume & sound */

_Static_assert(sizeof(settings_t) == CFG_SLOT_SIZE, "slot size");

static uint8_t sum8(const void *p, uint32_t n)
{
    const uint8_t *b = p; uint8_t s = 0;
    while (n--) s += *b++;
    return s;
}

static const settings_t *slot(uint32_t i)
{
    return (const settings_t *)(CFG_PAGE_ADDR + i * CFG_SLOT_SIZE);
}

static bool slot_used(const settings_t *s)   /* erased flash reads 0xFF */
{
    return s->magic != 0xFFFF;
}

static bool slot_valid(const settings_t *s)
{
    return s->magic == CFG_MAGIC && s->version == CFG_VERSION &&
           sum8(s, CFG_SLOT_SIZE) == 0;      /* checksum makes byte-sum 0 */
}

bool settings_load(settings_t *out)
{
    const settings_t *best = NULL;
    for (uint32_t i = 0; i < CFG_SLOTS; i++) {
        if (!slot_used(slot(i))) break;
        if (slot_valid(slot(i))) best = slot(i);
    }
    if (!best) return false;
    *out = *best;
    return true;
}

bool settings_save(const settings_t *in)
{
    settings_t s = *in;
    s.magic = CFG_MAGIC;
    s.version = CFG_VERSION;
    s.pad[0] = s.pad[1] = 0;
    s.checksum = 0;
    s.checksum = (uint8_t)(0u - sum8(&s, CFG_SLOT_SIZE));

    settings_t cur;                          /* skip identical writes (wear) */
    if (settings_load(&cur) && memcmp(&cur, &s, CFG_SLOT_SIZE) == 0)
        return true;

    uint32_t i = 0;
    while (i < CFG_SLOTS && slot_used(slot(i))) i++;

    HAL_FLASH_Unlock();
    bool ok = true;
    if (i >= CFG_SLOTS) {                    /* log full - recycle the page */
        FLASH_EraseInitTypeDef e = {0};
        uint32_t err;
        e.TypeErase = FLASH_TYPEERASE_PAGES;
        e.Banks     = FLASH_BANK_2;
        e.Page      = FLASH_PAGE_NB - 1;     /* page index within the bank */
        e.NbPages   = 1;
        ok = (HAL_FLASHEx_Erase(&e, &err) == HAL_OK);
        i = 0;
    }
    if (ok) {
        uint64_t w[CFG_SLOT_SIZE / 8];
        memcpy(w, &s, CFG_SLOT_SIZE);
        uint32_t addr = CFG_PAGE_ADDR + i * CFG_SLOT_SIZE;
        for (uint32_t k = 0; ok && k < CFG_SLOT_SIZE / 8; k++)
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                   addr + 8 * k, w[k]) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok;
}
