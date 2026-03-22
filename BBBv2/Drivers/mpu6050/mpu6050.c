#include "stm32f4xx_hal.h"
#include "mpu6050.h"

static uint8_t imu_addr;          /* I2C address set during init (shifted for HAL) */
static uint8_t imu_buffer[14];    /* raw burst-read: 6 accel + 2 temp + 6 gyro bytes */

/* Verify the device responds and identifies itself correctly */
static uint8_t MPU6050_Check(I2C_HandleTypeDef *hi2c)
{
    uint8_t who_am_i;

    if (HAL_I2C_Mem_Read(hi2c, imu_addr, MPU6050_WHO_AM_I,
        I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100) != HAL_OK)
        return 0;

    return (who_am_i == 0x68);   /* fixed ID for MPU-6050 */
}

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t address)
{
    imu_addr = address;

    if (!MPU6050_Check(hi2c))
        return 0;

    uint8_t config;

    /* PWR_MGMT_1: clear sleep bit, select PLL from X-axis gyro as clock source
       (more stable than the internal 8 MHz oscillator) */
    config = 0x01;
    HAL_I2C_Mem_Write(hi2c, imu_addr, MPU6050_PWR_ADDR,
                      I2C_MEMADD_SIZE_8BIT, &config, 1, 100);

    HAL_Delay(100);   /* wait for PLL to stabilise */

    /* ACCEL_CONFIG: AFS_SEL = 3 → ±16 g full-scale (LSB = 2048 counts/g) */
    config = 0x18;
    HAL_I2C_Mem_Write(hi2c, imu_addr, MPU6050_ACCEL_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &config, 1, 100);

    /* GYRO_CONFIG: FS_SEL = 3 → ±2000 dps full-scale (LSB = 16.4 counts/dps) */
    config = 0x18;
    HAL_I2C_Mem_Write(hi2c, imu_addr, MPU6050_GYRO_CONFIG,
                      I2C_MEMADD_SIZE_8BIT, &config, 1, 100);

    return 1;
}

uint8_t MPU6050_Read(I2C_HandleTypeDef *hi2c, sample_t *sample)
{
    /* Burst-read 14 bytes starting at ACCEL_XOUT_H.
       Register map: [0-5] accel XYZ, [6-7] temp (discarded), [8-13] gyro XYZ */
    if (HAL_I2C_Mem_Read(hi2c, imu_addr, MPU6050_DATA_ADDR,
        I2C_MEMADD_SIZE_8BIT, imu_buffer, 14, 100) != HAL_OK)
        return 0;

    /* Reconstruct 16-bit signed values from big-endian register pairs */
    sample->ax = (int16_t)((imu_buffer[0] << 8) | imu_buffer[1]);
    sample->ay = (int16_t)((imu_buffer[2] << 8) | imu_buffer[3]);
    sample->az = (int16_t)((imu_buffer[4] << 8) | imu_buffer[5]);

    /* imu_buffer[6..7] = temperature — skipped */
    sample->gx = (int16_t)((imu_buffer[8]  << 8) | imu_buffer[9]);
    sample->gy = (int16_t)((imu_buffer[10] << 8) | imu_buffer[11]);
    sample->gz = (int16_t)((imu_buffer[12] << 8) | imu_buffer[13]);

    return 1;
}
