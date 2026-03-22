#ifndef GPS_H
#define GPS_H

#include "stm32f4xx_hal_uart.h"
#include "data_types.h"

/* Start DMA-driven NMEA reception on the given UART.
   Must be called once before GPS_GetData. */
void GPS_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

/* Return an atomic snapshot of the latest parsed GPS data.
   Check gd.valid to determine whether a satellite fix is active. */
gps_data_t GPS_GetData(void);

#endif
