/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

#include "stm32f4xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "sdcard.h"
#include "gps.h"
#include "ssd1306.h"
#include "data_types.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
#define LOG_TYPE_IMU  0x01u   /* SD record tag: IMU sample */
#define LOG_TYPE_GPS  0x02u   /* SD record tag: GPS fix    */

typedef struct __attribute__((packed)) {
    float    latitude;
    float    longitude;
    float    speed;
    char     time[12];
    uint32_t timestamp_ms;
} gps_log_record_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define SAMPLE_RATE      40                               /* IMU samples per second */
#define BUFFER_SECONDS   60                               /* ring buffer depth */
#define BUFFER_SIZE      (SAMPLE_RATE * BUFFER_SECONDS)   /* 2400 samples */

/* Squared-magnitude thresholds (raw ADC units²); no sqrt needed */
#define ACCEL_THRESHOLD  15000000u
#define GYRO_THRESHOLD   3000000u

#define RTC_INIT_FLAG    0xABCE   /* sentinel stored in backup register to detect first boot */

#define GPS_POLL_MS      1000u    /* poll GPS once per second */
#define OLED_REFRESH_MS  200u     /* update OLED 5 times per second */

/* Sensor rail is active-low: RESET = power on, SET = power off */
#define SENSORS_POWER_ON()  HAL_GPIO_WritePin(Sensor_Toggle_GPIO_Port, \
                                               Sensor_Toggle_Pin, GPIO_PIN_RESET)
#define SENSORS_POWER_OFF() HAL_GPIO_WritePin(Sensor_Toggle_GPIO_Port, \
                                               Sensor_Toggle_Pin, GPIO_PIN_SET)
/* USER CODE END PD */

I2C_HandleTypeDef  hi2c1;   /* MPU-6050 */
RTC_HandleTypeDef  hrtc;
SPI_HandleTypeDef  hspi1;   /* SD card  */
TIM_HandleTypeDef  htim2;   /* 1 ms system tick + sample trigger */
UART_HandleTypeDef huart2;  /* GPS NMEA RX (DMA) */
DMA_HandleTypeDef  hdma_usart2_rx;

/* USER CODE BEGIN PV */
extern char sentence_buffer[100];   /* filled by GPS DMA ISR */

volatile uint8_t  sample_trigger     = 0;   /* counts TIM2 ticks toward next IMU sample */
volatile uint32_t system_time_ms     = 0;   /* free-running ms counter, incremented in TIM2 ISR */

sample_t buffer[BUFFER_SIZE];   /* circular ring buffer of IMU samples */

volatile uint8_t sample_request_flag = 0;   /* set by ISR, cleared in main loop */
uint8_t  crash_trigger_flag          = 0;   /* set when crash threshold exceeded */
uint8_t  buffer_freeze_flag          = 0;   /* set when post-crash capture is complete */

static uint8_t crash_displayed = 0;   /* prevents overwriting the crash screen on OLED */

uint16_t write_index    = 0;   /* next slot to write in the ring buffer */
uint16_t crash_index    = 0;   /* ring buffer position at moment of crash detection */

uint64_t accel_mag = 0;   /* squared magnitude of accelerometer vector */
uint64_t gyro_mag  = 0;   /* squared magnitude of gyroscope vector */

int32_t ax, ay, az;
int32_t gx, gy, gz;

uint8_t  crash_counter      = 0;   /* consecutive samples above threshold */
uint16_t post_crash_counter = 0;   /* samples collected after crash trigger */

RTC_TimeTypeDef rtc_time_ref;   /* last RTC snapshot, refreshed at 1 Hz */
RTC_DateTypeDef rtc_date_ref;

volatile uint32_t last_rtc_sync   = 0;
volatile uint32_t rtc_sync_ms_ref = 0;

static sample_t last_valid_sample = {0};   /* used as fallback if an I2C read fails */

uint8_t  sd_buffer[512];     /* one SD sector staging buffer */
uint16_t sd_index       = 0;
uint32_t current_sector = 0;
uint32_t base_sector    = 2048;   /* first writable sector (avoids MBR/FAT area) */
uint8_t  logging_done   = 0;

static gps_log_record_t last_gps_record  = {0};
static uint8_t          gps_fix_valid    = 0;
static uint32_t         last_gps_poll_ms = 0;
static uint32_t         last_oled_ms     = 0;

/* Tracks actual samples stored so far; prevents logging uninitialised slots
   when the buffer has never been fully filled. */
static uint16_t samples_written = 0;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN 0 */
/* Latch current RTC time and date into global references */
void Update_Time_Reference(void)
{
    /* Date must be read after Time to unlock the shadow registers correctly */
    HAL_RTC_GetTime(&hrtc, &rtc_time_ref, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &rtc_date_ref, RTC_FORMAT_BIN);
}

/* Write a type-tagged record into the 512-byte SD staging buffer.
   Flushes the sector to SD automatically when full. */
static void SD_WriteTaggedRecord(uint8_t type, const void *payload,
                                  uint16_t payload_len, uint8_t *sd_error)
{
    uint16_t record_len = 1u + payload_len;   /* tag byte + payload */
    if (sd_index + record_len > 512u)         /* sector full — flush first */
    {
        if (!SD_Write(current_sector, sd_buffer)) { *sd_error = 1; return; }
        current_sector++;
        sd_index = 0;
        memset(sd_buffer, 0, 512u);
    }
    sd_buffer[sd_index++] = type;
    memcpy(&sd_buffer[sd_index], payload, payload_len);
    sd_index += payload_len;
}

/* snprintf-safe float formatter for targets without printf float support */
static void __attribute__((unused)) Format_Float_As_Int(char *buf, size_t buf_len,
                                                         float val, uint8_t decimals)
{
    int   neg     = (val < 0.0f);
    float abs_val = neg ? -val : val;
    long  ip      = (long)abs_val;
    float frac_f  = abs_val - (float)ip;
    unsigned long scale = 1;
    for (uint8_t i = 0; i < decimals; i++) scale *= 10u;
    unsigned long fp = (unsigned long)(frac_f * (float)scale + 0.5f);
    if (fp >= scale) { fp = 0; ip++; }   /* handle rounding carry */
    snprintf(buf, buf_len, "%s%ld.%0*lu", neg ? "-" : "", ip, (int)decimals, fp);
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();   /* required before touching RTC/backup registers */

    /* Drive sensor rail OFF before LSE wait — on a cold power cycle GPIO is not yet
       initialised, so Sensor_Toggle (PB10) floats and can accidentally power sensors.
       Uncontrolled sensor power-on corrupts the I2C bus before MX_I2C1_Init runs. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    {
        GPIO_InitTypeDef g = {0};
        g.Pin   = GPIO_PIN_10;
        g.Mode  = GPIO_MODE_OUTPUT_PP;
        g.Pull  = GPIO_NOPULL;
        g.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOB, &g);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);   /* sensors OFF (active-low) */

    MX_RTC_Init();

    /* Only set default time/date on the very first boot (backup register is blank) */
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != RTC_INIT_FLAG)
    {
        RTC_TimeTypeDef sTime = {0};
        RTC_DateTypeDef sDate = {0};
        sTime.Hours = 6; sTime.Minutes = 43; sTime.Seconds = 0;
        sDate.Date  = 22; sDate.Month   = 3; sDate.Year    = 26;
        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, RTC_INIT_FLAG);   /* mark as initialised */
    }
    /* USER CODE END SysInit */

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    SENSORS_POWER_ON();
    HAL_Delay(50);   /* MPU6050 requires ≥30 ms after power rail rises */

    /* Sensor power-up can glitch SDA/SCL and leave the I2C peripheral in an error
       state. Re-init after the rail is stable so the first OLED write starts clean. */
    HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();

    /* Init OLED first so it can show status regardless of IMU state */
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("System Init...", Font_7x10, White);
    ssd1306_UpdateScreen();

    {
        uint16_t imu_retry = 0;
        while (!MPU6050_Init(&hi2c1, MPU6050_ADDR_LOW))
        {
            HAL_Delay(10);
            if (++imu_retry >= 50)   /* ~500 ms with no response — bus is stuck */
            {
                HAL_I2C_DeInit(&hi2c1);
                MX_I2C1_Init();
                imu_retry = 0;
            }
        }
    }
    /* Re-init OLED in case the IMU retry loop triggered I2C bus recovery */
    ssd1306_Init();

    GPS_Init(&huart2, &hdma_usart2_rx);
    HAL_TIM_Base_Start_IT(&htim2);   /* start 1 ms tick */

    /* Snapshot references so relative timers start from zero */
    last_rtc_sync    = system_time_ms;
    rtc_sync_ms_ref  = system_time_ms;
    last_gps_poll_ms = system_time_ms;
    last_oled_ms     = system_time_ms;
    Update_Time_Reference();

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("IMU OK", Font_7x10, White);
    ssd1306_UpdateScreen();
    HAL_Delay(500);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("System Ready", Font_7x10, White);
    ssd1306_UpdateScreen();

    SD_DriverInit(&hspi1);
    if (SD_Init())
    {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("SD OK", Font_7x10, White);
    }
    else
    {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("SD FAIL", Font_7x10, White);
    }
    ssd1306_UpdateScreen();
    HAL_Delay(1000);
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */

        uint32_t now = system_time_ms;   /* single snapshot avoids skew across checks */

        /* ---------------------------------------------------------------- */
        /* RTC SYNC — 1 Hz                                                  */
        /* ---------------------------------------------------------------- */
        if ((now - last_rtc_sync) >= 1000u)
        {
            last_rtc_sync   = now;
            rtc_sync_ms_ref = now;
            Update_Time_Reference();
        }

        /* ---------------------------------------------------------------- */
        /* GPS POLL — 1 Hz                                                  */
        /* ---------------------------------------------------------------- */
        if ((now - last_gps_poll_ms) >= GPS_POLL_MS)
        {
            last_gps_poll_ms = now;

            gps_data_t gd = GPS_GetData();
            gps_fix_valid  = gd.valid;

            last_gps_record.latitude     = gd.latitude;
            last_gps_record.longitude    = gd.longitude;
            last_gps_record.speed        = gd.speed;
            last_gps_record.timestamp_ms = now;

            /* Copy time string safely without overflowing the destination */
            size_t clen = sizeof(last_gps_record.time) < sizeof(gd.time)
                          ? sizeof(last_gps_record.time) : sizeof(gd.time);

            memset(last_gps_record.time, 0, sizeof(last_gps_record.time));
            memcpy(last_gps_record.time, gd.time, clen - 1);   /* leave null terminator */
        }

        /* ---------------------------------------------------------------- */
        /* OLED UPDATE — 5 Hz (every 200 ms)                                */
        /* ---------------------------------------------------------------- */
        if ((now - last_oled_ms) >= OLED_REFRESH_MS)
        {
            last_oled_ms = now;

            /* Skip normal status display if the crash screen is showing */
            if (!crash_displayed)
            {
                ssd1306_Fill(Black);
                char line[24];

                snprintf(line, sizeof(line), "A:%6d%6d%6d",
                         (int)last_valid_sample.ax,
                         (int)last_valid_sample.ay,
                         (int)last_valid_sample.az);
                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString(line, Font_6x8, White);

                snprintf(line, sizeof(line), "G:%6d%6d%6d",
                         (int)last_valid_sample.gx,
                         (int)last_valid_sample.gy,
                         (int)last_valid_sample.gz);
                ssd1306_SetCursor(0, 8);
                ssd1306_WriteString(line, Font_6x8, White);

                snprintf(line, sizeof(line), "V:%d T:%.8s",
                         (int)gps_fix_valid, last_gps_record.time);
                ssd1306_SetCursor(0, 16);
                ssd1306_WriteString(line, Font_6x8, White);

                snprintf(line, sizeof(line), "T:%02d:%02d:%02d",
                         rtc_time_ref.Hours,
                         rtc_time_ref.Minutes,
                         rtc_time_ref.Seconds);
                ssd1306_SetCursor(0, 24);
                ssd1306_WriteString(line, Font_6x8, White);

                snprintf(line, sizeof(line), "D:%02d/%02d/%02d",
                         rtc_date_ref.Date,
                         rtc_date_ref.Month,
                         rtc_date_ref.Year);
                ssd1306_SetCursor(0, 32);
                ssd1306_WriteString(line, Font_6x8, White);

                snprintf(line, sizeof(line), "UP:%lu s",
                         (unsigned long)(now / 1000u));
                ssd1306_SetCursor(0, 40);
                ssd1306_WriteString(line, Font_6x8, White);

                if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
                {
                    HAL_I2C_DeInit(&hi2c1);
                    MX_I2C1_Init();
                    ssd1306_Init();
                }

                ssd1306_UpdateScreen();
            }
        }

        /* ---------------------------------------------------------------- */
        /* IMU SAMPLING @ 40 Hz                                             */
        /* ---------------------------------------------------------------- */
        if (sample_request_flag)
        {
            sample_request_flag = 0;
            uint32_t read_start = system_time_ms;

            /* Read sensor; fall back to last good values on I2C failure */
            if (MPU6050_Read(&hi2c1, &buffer[write_index]))
            {
                last_valid_sample = buffer[write_index];
            }
            else
            {
                buffer[write_index].ax = last_valid_sample.ax;
                buffer[write_index].ay = last_valid_sample.ay;
                buffer[write_index].az = last_valid_sample.az;
                buffer[write_index].gx = last_valid_sample.gx;
                buffer[write_index].gy = last_valid_sample.gy;
                buffer[write_index].gz = last_valid_sample.gz;
            }

            if ((system_time_ms - read_start) > 10)
                {
                    HAL_I2C_DeInit(&hi2c1);
                    MX_I2C1_Init();
                }

            buffer[write_index].timestamp = system_time_ms;

            /* Startup grace period (5 s): fill buffer but suppress crash detection
               to avoid false triggers while the sensor settles. */
            if (system_time_ms < 5000)
            {
                write_index = (write_index + 1) % BUFFER_SIZE;
                if (samples_written < BUFFER_SIZE) samples_written++;
                crash_counter = 0;
            }

            else if (!crash_trigger_flag)
            {
                /* Compute squared magnitudes (avoids costly sqrt) */
                ax = buffer[write_index].ax;
                ay = buffer[write_index].ay;
                az = buffer[write_index].az;
                gx = buffer[write_index].gx;
                gy = buffer[write_index].gy;
                gz = buffer[write_index].gz;

                accel_mag = (uint64_t)((int64_t)ax*ax + (int64_t)ay*ay + (int64_t)az*az);
                gyro_mag  = (uint64_t)((int64_t)gx*gx + (int64_t)gy*gy + (int64_t)gz*gz);

                /* Require 3 consecutive over-threshold samples (~75 ms) to
                   confirm a real crash and reject single-sample spikes. */
                if (accel_mag >= ACCEL_THRESHOLD || gyro_mag >= GYRO_THRESHOLD)
                {
                    crash_counter++;
                    if (crash_counter >= 3)
                    {
                        crash_trigger_flag = 1;
                        crash_index        = write_index;
                        crash_counter      = 0;

                        /* Show crash alert once; do not overwrite it during OLED refresh */
                        if (!crash_displayed)
                        {
                            crash_displayed = 1;

                            ssd1306_Fill(Black);
                            ssd1306_SetCursor(0, 0);
                            ssd1306_WriteString("CRASH!", Font_7x10, White);
                            ssd1306_UpdateScreen();
                        }
                    }
                }
                else
                {
                    crash_counter = 0;
                }

                write_index = (write_index + 1) % BUFFER_SIZE;
                if (samples_written < BUFFER_SIZE) samples_written++;
            }
            else
            {
                /* Post-crash: capture 2 s (80 samples @ 40 Hz) of data then freeze */
                if (post_crash_counter < 80u)
                {
                    post_crash_counter++;
                    write_index = (write_index + 1) % BUFFER_SIZE;
                    if (samples_written < BUFFER_SIZE) samples_written++;
                }
                else
                {
                    buffer_freeze_flag = 1;   /* signal main loop to begin SD logging */
                }
            }
        }

        /* ---------------------------------------------------------------- */
        /* SD LOGGING                                                        */
        /* ---------------------------------------------------------------- */
        if (buffer_freeze_flag)
        {
            ssd1306_Fill(Black);
            ssd1306_SetCursor(0, 0);
            ssd1306_WriteString("Logging...", Font_7x10, White);
            ssd1306_UpdateScreen();

            /* Use actual sample count so we never log uninitialised buffer slots */
            uint16_t log_count = samples_written;

            /* Determine oldest sample in the ring buffer */
            uint16_t start_index;
            if (samples_written < BUFFER_SIZE)
                start_index = 0;                                  /* buffer not yet wrapped */
            else
                start_index = (crash_index + 1u) % BUFFER_SIZE;  /* wrapped: oldest is one past crash */

            uint16_t current  = start_index;
            uint8_t  sd_error = 0;

            sd_index       = 0;
            current_sector = base_sector;
            memset(sd_buffer, 0, sizeof(sd_buffer));

            /* Prepend a GPS record so the log starts with a known position */
            if (gps_fix_valid)
            {
                SD_WriteTaggedRecord(LOG_TYPE_GPS, &last_gps_record,
                                     sizeof(gps_log_record_t), &sd_error);
            }

            /* Write all IMU samples; interleave a GPS record every 40 samples (1 s) */
            for (uint32_t i = 0; i < log_count && !sd_error; i++)
            {
                if (i > 0u && (i % 40u) == 0u)
                {
                    SD_WriteTaggedRecord(LOG_TYPE_GPS, &last_gps_record,
                                         sizeof(gps_log_record_t), &sd_error);
                    if (sd_error) break;
                }
                SD_WriteTaggedRecord(LOG_TYPE_IMU, &buffer[current],
                                     sizeof(sample_t), &sd_error);
                current = (current + 1u) % BUFFER_SIZE;
            }

            /* Flush any remaining bytes in the partial final sector */
            if (!sd_error && sd_index > 0u)
            {
                memset(&sd_buffer[sd_index], 0, 512u - sd_index);   /* zero-pad to sector boundary */
                if (!SD_Write(current_sector, sd_buffer)) sd_error = 1;
            }

            if (sd_error)
            {
                ssd1306_Fill(Black);
                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString("SD ERROR", Font_7x10, White);
                ssd1306_UpdateScreen();
            }

            /* Advance base_sector past the maximum possible log size so the next
               event doesn't overwrite this one. */
            uint32_t max_bytes =
                (uint32_t)BUFFER_SIZE * (1u + sizeof(sample_t))
              + (uint32_t)(BUFFER_SIZE / 40u + 2u) * (1u + sizeof(gps_log_record_t));
            base_sector += (max_bytes + 511u) / 512u;

            /* Reset all crash/logging state for the next event */
            post_crash_counter = 0;
            crash_trigger_flag = 0;
            buffer_freeze_flag = 0;
            crash_counter      = 0;
            write_index        = 0;
            logging_done       = 1;
            samples_written    = 0;

            ssd1306_Fill(Black);
            ssd1306_SetCursor(0, 0);
            ssd1306_WriteString("Log Saved", Font_7x10, White);
            ssd1306_UpdateScreen();

            crash_displayed = 0;
        }

        /* USER CODE END 3 */
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* HSE (external crystal) + LSI for RTC; PLL from HSE */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    /* 25 MHz HSE → /25 × 168 / 2 = 84 MHz SYSCLK */
    RCC_OscInitStruct.PLL.PLLM       = 25;
    RCC_OscInitStruct.PLL.PLLN       = 168;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;    /* HCLK  = 84 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;      /* APB1  = 42 MHz (I2C, UART) */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      /* APB2  = 84 MHz (SPI, TIM) */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;   /* 400 kHz fast mode */
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_RTC_Init(void)
{
    hrtc.Instance            = RTC;
    hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
    /* Async = 128, Sync = 250 → 1 Hz from ~32 kHz LSI */
    hrtc.Init.AsynchPrediv   = 127;
    hrtc.Init.SynchPrediv    = 249;
    hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
    HAL_RTC_Init(&hrtc);   /* non-fatal: RTC time display is informational only */
}

static void MX_SPI1_Init(void)
{
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;   /* SPI mode 0 */
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;   /* ~328 kHz — slow for SD init */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim2.Instance               = TIM2;
    /* 84 MHz / (83+1) / (999+1) = 1 kHz → 1 ms period */
    htim2.Init.Prescaler         = 83;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 9600;   /* standard GPS NMEA baud rate */
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_RX;   /* GPS is receive-only */
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* DMA1 Stream5 services USART2_RX */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Default SD CS high (deselected) and sensor rail off until explicitly enabled */
    HAL_GPIO_WritePin(CS_GPIO_Port,            CS_Pin,            GPIO_PIN_SET);
    HAL_GPIO_WritePin(Sensor_Toggle_GPIO_Port, Sensor_Toggle_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = Sensor_Toggle_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(Sensor_Toggle_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        system_time_ms++;

        /* Generate 40 Hz sample requests: every 25 ms ticks */
        if (!buffer_freeze_flag)
        {
            sample_trigger++;
            if (sample_trigger >= 25u)
            {
                sample_request_flag = 1;
                sample_trigger      = 0;
            }
        }
    }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}   /* halt; attach debugger or observe watchdog reset */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
