/* bsp.h - board handles & pin helpers shared by the App modules */
#ifndef BSP_H
#define BSP_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Peripheral handles defined in main.c */
extern I2C_HandleTypeDef hi2c2;
extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch1;
extern TIM_HandleTypeDef htim2;   /* rotary encoder (TI12) */
extern TIM_HandleTypeDef htim6;   /* DAC sample clock, 16 kHz */

/* I2C 7-bit addresses, pre-shifted for the HAL (<<1) */
#define ADDR_SH1106   (0x3C << 1)
#define ADDR_RDA5807  (0x10 << 1)   /* sequential-access address */
#define ADDR_DS3231   (0x68 << 1)

#endif /* BSP_H */
