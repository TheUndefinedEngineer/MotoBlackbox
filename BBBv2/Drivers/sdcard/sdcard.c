#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "main.h"
#include <stdbool.h>

static SPI_HandleTypeDef *sd_spi;
static bool sd_high_capacity = false;   /* true = SDHC/SDXC (sector addressing), false = SDSC (byte addressing) */

void SD_DriverInit(SPI_HandleTypeDef *hspi)
{
    sd_spi = hspi;
}

/* Single-byte SPI exchange; SD protocol requires clocking out 0xFF when reading */
static uint8_t SPI_TxRx(uint8_t tx)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(sd_spi, &tx, &rx, 1, 100);
    return rx;
}

static void SD_Select(void)
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);   /* active low */
}

static void SD_Deselect(void)
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/* SD spec requires ≥74 clock pulses with CS deasserted before CMD0 */
static void SD_SendClockTrain(void)
{
    SD_Deselect();
    for (int i = 0; i < 10; i++)
        SPI_TxRx(0xFF);   /* 10 × 8 bits = 80 clocks */
}

/* Poll until the card releases DO (0xFF = ready); returns false on timeout */
static bool SD_WaitReady(uint32_t timeout)
{
    uint8_t res;
    uint32_t t = HAL_GetTick();

    do
    {
        res = SPI_TxRx(0xFF);
        if (res == 0xFF)
            return true;
    } while ((HAL_GetTick() - t) < timeout);

    return false;
}

/* Send a 6-byte SD command frame and return the R1 response byte.
   Caller must assert CS before calling and deassert it afterwards. */
static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t response;
    uint16_t wait = 0;

    if (!SD_WaitReady(50))
        return 0xFF;

    SPI_TxRx(0xFF);           /* one idle byte before command frame */

    SPI_TxRx(0x40 | cmd);    /* start bit + command index */
    SPI_TxRx(arg >> 24);
    SPI_TxRx(arg >> 16);
    SPI_TxRx(arg >> 8);
    SPI_TxRx(arg);
    SPI_TxRx(crc);            /* CRC7 + stop bit; only checked for CMD0 and CMD8 */

    /* R1 response: MSB is 0 when valid, so poll until it clears */
    do
    {
        response = SPI_TxRx(0xFF);
        wait++;
    } while ((response & 0x80) && wait < 1000);

    return response;
}

bool SD_Init(void)
{
    uint8_t response;
    uint8_t r7[4];    /* CMD8 R7 response: voltage range + echo-back pattern */
    uint8_t ocr[4];   /* CMD58 OCR register: capacity and voltage info */
    uint16_t wait;

    HAL_Delay(20);   /* allow card power rail to stabilise */

    SD_Deselect();
    SD_SendClockTrain();

    /* CMD0 — reset card into SPI mode; expect R1 = 0x01 (idle state) */
    wait = 0;
    do
    {
        SD_Select();
        response = SD_SendCommand(0, 0, 0x95);   /* CRC hardcoded for CMD0 */
        SD_Deselect();
        SPI_TxRx(0xFF);
        wait++;
    } while (response != 0x01 && wait < 100);

    if (response != 0x01)
        return false;

    /* CMD8 — verify 2.7–3.6 V support and confirm SDv2 card.
       Arg: VHS = 0x01 (2.7–3.6 V), check pattern = 0xAA */
    SD_Select();
    response = SD_SendCommand(8, 0x000001AA, 0x87);   /* CRC hardcoded for CMD8 */
    if (response != 0x01)
    {
        SD_Deselect();
        return false;   /* SDv1 or MMC — not supported */
    }

    for (int i = 0; i < 4; i++)
        r7[i] = SPI_TxRx(0xFF);

    SD_Deselect();
    SPI_TxRx(0xFF);

    if (r7[3] != 0xAA)
        return false;   /* echo-back mismatch — card unusable */

    /* ACMD41 — start card initialisation; HCS bit set to request SDHC support.
       ACMD41 = CMD55 (app command prefix) followed by CMD41. */
    wait = 0;
    do
    {
        SD_Select();
        response = SD_SendCommand(55, 0, 0xFF);   /* CMD55: next command is ACMD */
        SD_Deselect();
        SPI_TxRx(0xFF);

        if (response < 0x01)
            return false;

        SD_Select();
        response = SD_SendCommand(41, 0x40000000, 0xFF);   /* HCS bit [30] set */
        SD_Deselect();
        SPI_TxRx(0xFF);

        wait++;
    } while (response != 0x00 && wait < 500);   /* 0x00 = initialisation complete */

    if (response != 0x00)
        return false;

    /* CMD58 — read OCR to determine whether card is SDHC/SDXC (bit 30 = CCS) */
    SD_Select();
    response = SD_SendCommand(58, 0, 0xFF);
    if (response != 0x00)
    {
        SD_Deselect();
        return false;
    }

    for (int i = 0; i < 4; i++)
        ocr[i] = SPI_TxRx(0xFF);

    SD_Deselect();
    SPI_TxRx(0xFF);

    if (ocr[0] & 0x40)
        sd_high_capacity = true;   /* CCS = 1: SDHC/SDXC uses sector addressing */

    return true;
}

/* Write one 512-byte block at the given sector address; retries up to 3 times */
bool SD_Write(uint32_t sector, const uint8_t *data)
{
    uint32_t address;
    uint8_t response;
    uint8_t retry = 0;

    while (retry < 3)
    {
        SD_Select();

        if (!SD_WaitReady(500))
        {
            SD_Deselect();
            retry++;
            continue;
        }

        /* SDHC: address is sector number; SDSC: address is byte offset */
        address = sd_high_capacity ? sector : sector * 512;

        /* CMD24 — write single block */
        response = SD_SendCommand(24, address, 0xFF);
        if (response != 0x00)
        {
            SD_Deselect();
            SPI_TxRx(0xFF);
            retry++;
            continue;
        }

        SPI_TxRx(0xFE);   /* data start token */

        for (int i = 0; i < 512; i++)
            SPI_TxRx(data[i]);

        SPI_TxRx(0xFF);   /* dummy CRC (2 bytes, not checked) */
        SPI_TxRx(0xFF);

        /* Data response token: bits [4:0] = 0x05 means data accepted */
        response = SPI_TxRx(0xFF);
        if ((response & 0x1F) != 0x05)
        {
            SD_Deselect();
            retry++;
            continue;
        }

        if (!SD_WaitReady(500))   /* wait for internal write to complete */
        {
            SD_Deselect();
            retry++;
            continue;
        }

        SD_Deselect();
        SPI_TxRx(0xFF);   /* one idle clock after deselect */

        return true;
    }

    return false;
}
