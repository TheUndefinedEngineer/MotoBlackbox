#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h"
#include "data_types.h"

/* I2C addresses pre-shifted for the HAL (7-bit addr << 1).
   AD0 pin low → 0x68, AD0 pin high → 0x69 */
#define MPU6050_ADDR_LOW  (0x68 << 1)
#define MPU6050_ADDR_HIGH (0x69 << 1)

/* Registers */
#define MPU6050_WHO_AM_I      0x75   /* read-only device ID, always returns 0x68 */
#define MPU6050_PWR_ADDR      0x6B   /* PWR_MGMT_1: sleep, clock source           */
#define MPU6050_ACCEL_CONFIG  0x1C   /* AFS_SEL bits [4:3] set full-scale range   */
#define MPU6050_GYRO_CONFIG   0x1B   /* FS_SEL  bits [4:3] set full-scale range   */
#define MPU6050_DATA_ADDR     0x3B   /* ACCEL_XOUT_H: start of 14-byte data block */

/* Initialise the MPU-6050 at the given address.
   Returns 1 on success, 0 if the device does not respond or WHO_AM_I fails. */
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, uint8_t address);

/* Burst-read one sample (accel + gyro) into *sample.
   Returns 1 on success, 0 on I2C error. */
uint8_t MPU6050_Read(I2C_HandleTypeDef *hi2c, sample_t *sample);

#endif
