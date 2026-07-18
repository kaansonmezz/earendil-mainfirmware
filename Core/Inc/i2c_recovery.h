#ifndef I2C_RECOVERY_H
#define I2C_RECOVERY_H

#include "stm32h7xx_hal.h"

/* I2C bus recovery: unstick a locked bus (slave holding SDA low).
 * Performs 9-clock pulse recovery on SCL, then re-initialises the
 * I2C peripheral via HAL_I2C_DeInit / HAL_I2C_Init.
 * Returns HAL_OK if bus is recovered (SDA reads high after recovery). */
HAL_StatusTypeDef I2C_BusRecovery(I2C_HandleTypeDef *hi2c);

/* I2C peripheral hard-reset via RCC force/release reset + HAL re-init.
 * Use when the peripheral is stuck in BUSY state. */
HAL_StatusTypeDef I2C_BusResetAndReinit(I2C_HandleTypeDef *hi2c);

/* Combined: try to recover a stuck bus.  First attempt 9-clock recovery,
 * if that fails do a full peripheral reset.  Returns HAL_OK on success. */
HAL_StatusTypeDef I2C_RecoverBus(I2C_HandleTypeDef *hi2c);

#endif /* I2C_RECOVERY_H */
