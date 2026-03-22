#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "data_types.h"

#define DMA_SIZE 1024

volatile gps_data_t gps_data;   /* written in DMA callback, read in main loop */

uint8_t dma_buffer[DMA_SIZE];   /* circular DMA receive buffer */

char    sentence_buffer[100];   /* accumulates one NMEA sentence between '$' and '\n' */
uint8_t sindex     = 0;
uint8_t collecting = 0;         /* 1 while assembling a sentence */

static void GPS_Process(void);

static UART_HandleTypeDef *gps_uart;
static DMA_HandleTypeDef  *gps_dma;

/* Atomically snapshot gps_data for safe use in the main loop */
gps_data_t GPS_GetData(void)
{
    gps_data_t copy;
    __disable_irq();
    copy = gps_data;
    __enable_irq();
    return copy;
}

void GPS_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma)
{
    gps_uart = huart;
    gps_dma  = hdma;
    HAL_UARTEx_ReceiveToIdle_DMA(gps_uart, dma_buffer, DMA_SIZE);
    __HAL_DMA_DISABLE_IT(gps_dma, DMA_IT_HT);   /* suppress half-transfer interrupt; use idle-line only */
}

/* Convert NMEA ddmm.mmmm format to decimal degrees */
static float convert_to_decimal(float raw)
{
    int   degrees = (int)(raw / 100.0f);
    float minutes = raw - (degrees * 100.0f);
    return (float)degrees + (minutes / 60.0f);
}

/* Called by HAL when the UART goes idle or the DMA buffer fills */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != gps_uart) return;

    for (int i = 0; i < size; i++)
    {
        uint8_t byte = dma_buffer[i];

        if (byte == '$')
        {
            sindex     = 0;
            collecting = 1;
        }

        if (collecting)
        {
            if (sindex < sizeof(sentence_buffer) - 1)
            {
                sentence_buffer[sindex++] = byte;
            }
            else
            {
                /* Sentence too long — discard and wait for next '$' */
                collecting = 0;
                sindex     = 0;
            }
        }

        if (byte == '\n' && collecting)
        {
            sentence_buffer[sindex] = '\0';
            collecting = 0;
            sindex     = 0;
            GPS_Process();
        }
    }

    /* Restart DMA for next burst */
    HAL_UARTEx_ReceiveToIdle_DMA(gps_uart, dma_buffer, DMA_SIZE);
    __HAL_DMA_DISABLE_IT(gps_dma, DMA_IT_HT);
}

static void GPS_Process(void)
{
    char local_copy[100];
    strncpy(local_copy, sentence_buffer, sizeof(local_copy) - 1);
    local_copy[sizeof(local_copy) - 1] = '\0';

    /* Accept both $GPRMC (single-constellation) and $GNRMC (multi-constellation) */
    if (strncmp(local_copy, "$GPRMC", 6) != 0 &&
        strncmp(local_copy, "$GNRMC", 6) != 0)
        return;

    uint8_t    field = 0;
    char      *token;
    uint8_t    valid = 0;
    gps_data_t temp  = {0};   /* zero-init ensures lat/lon/speed stay 0 on no-fix */

    token = strtok(local_copy, ",");

    while (token != NULL)
    {
        switch (field)
        {
            case 1:
                /* UTC time string, e.g. "123519.00" — present even without a fix */
                strncpy(temp.time, token, sizeof(temp.time) - 1);
                temp.time[sizeof(temp.time) - 1] = '\0';
                break;

            case 2:
                /* 'A' = active fix, 'V' = void; lat/lon/speed only parsed when valid */
                valid      = (token[0] == 'A') ? 1u : 0u;
                temp.valid = valid;
                break;

            case 3:
                if (valid) temp.latitude = convert_to_decimal(atof(token));
                break;

            case 4:
                if (valid && token[0] == 'S') temp.latitude = -temp.latitude;
                break;

            case 5:
                if (valid) temp.longitude = convert_to_decimal(atof(token));
                break;

            case 6:
                if (valid && token[0] == 'W') temp.longitude = -temp.longitude;
                break;

            case 7:
                if (valid) temp.speed = atof(token);   /* speed over ground, knots */
                break;

            default:
                break;
        }

        token = strtok(NULL, ",");
        field++;
    }

    /* Always publish temp (not only on fix) so time and valid are always current */
    __disable_irq();
    gps_data = temp;
    __enable_irq();
}
