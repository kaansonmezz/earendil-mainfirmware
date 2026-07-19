#ifndef I2C_RECOVERY_H
#define I2C_RECOVERY_H

#include "stm32h7xx_hal.h"

/* ── I2C bus recovery ──────────────────────────────────────────────────────
 * Handles three fault classes:
 *   A. Physical bus stuck: SDA low or SCL low.
 *   B. Physical lines idle but STM32 I2C peripheral/HAL state stuck.
 *   C. Magnetometer internal state failure (handled by mag driver).
 *
 * Recovery affects both QMC5883L and MPU9250 on shared I2C1. */

/* Combined recovery entry point.  Selects the appropriate recovery
 * strategy based on line states and HAL error.
 * Returns HAL_OK on success. */
HAL_StatusTypeDef I2C_BusRecovery(I2C_HandleTypeDef *hi2c);

/* Full peripheral reset (RCC force/release + HAL re-init).
 * Use when the peripheral is stuck in BUSY state. */
HAL_StatusTypeDef I2C_BusResetAndReinit(I2C_HandleTypeDef *hi2c);

/* Alias for I2C_BusRecovery (backward compatibility). */
HAL_StatusTypeDef I2C_RecoverBus(I2C_HandleTypeDef *hi2c);

/* Returns non-zero if a recovery operation is currently in progress.
 * Callers should avoid initiating nested recovery. */
uint8_t I2C_RecoveryInProgress(void);

/* Invalidate MPU9250 address cache after shared-bus recovery.
 * Defined in imu_mpu9250.c. */
extern void IMU_MPU9250_InvalidateCache(void);

#endif /* I2C_RECOVERY_H */
