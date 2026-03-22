#ifndef SDCARD_H
#define SDCARD_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal_spi.h"

/* Store the SPI handle for use by all SD functions.
   Must be called before SD_Init. */
void SD_DriverInit(SPI_HandleTypeDef *hspi);

/* Initialise the SD card over SPI (supports SDSC and SDHC/SDXC).
   Returns true on success, false if the card is absent or unsupported. */
bool SD_Init(void);

/* Write one 512-byte block to the given sector address.
   Returns true on success; retries up to 3 times on failure. */
bool SD_Write(uint32_t sector, const uint8_t *data);

#endif
