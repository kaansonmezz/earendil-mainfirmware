/**
  ******************************************************************************
  * @file    mpu9250.h
  * @brief   MPU9250 IMU driver header.
  *          Migrated from STM32F411 to STM32H723ZG.
  *          This header is fully portable across STM32 HAL targets.
  ******************************************************************************
  */

#ifndef MPU9250_H
#define MPU9250_H

#include "main.h"

/**
 * @brief  MPU9250 data structure holding scaled sensor readings,
 *         calibration offsets, and computed orientation angles.
 */
typedef struct {
    /* Accelerometer readings in [g] */
    float Accel_X;
    float Accel_Y;
    float Accel_Z;

    /* Gyroscope readings in [deg/s], offset-corrected */
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;

    /* Gyroscope static bias offsets computed during calibration [deg/s] */
    float Gyro_X_Offset;
    float Gyro_Y_Offset;
    float Gyro_Z_Offset;

    /* Complementary-filter fused orientation angles [degrees] */
    float Roll;
    float Pitch;
    float Yaw;
} MPU9250_t;

/* ---- Public API --------------------------------------------------------- */

/**
 * @brief  Initialize the MPU9250 sensor (wake-up, sample rate, FSR config).
 * @param  I2Cx  Pointer to the HAL I2C handle.
 */
void MPU9250_Init(I2C_HandleTypeDef *I2Cx);

/**
 * @brief  Collect 200 static samples and compute gyroscope DC bias offsets.
 * @param  I2Cx       Pointer to the HAL I2C handle.
 * @param  DataStruct Pointer to the MPU9250 data structure.
 */
void MPU9250_Calibrate(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);

/**
 * @brief  Read raw accelerometer and gyroscope registers and convert to SI.
 * @param  I2Cx       Pointer to the HAL I2C handle.
 * @param  DataStruct Pointer to the MPU9250 data structure.
 * @retval 1 on success, 0 on I2C error.
 */
uint8_t MPU9250_Read_Accel_Gyro(I2C_HandleTypeDef *I2Cx, MPU9250_t *DataStruct);

/**
 * @brief  Run the complementary filter to update Roll, Pitch, Yaw.
 * @param  DataStruct Pointer to the MPU9250 data structure.
 * @param  dt         Time delta in seconds since last call.
 */
void MPU9250_Calculate_Angles(MPU9250_t *DataStruct, float dt);

#endif /* MPU9250_H */
