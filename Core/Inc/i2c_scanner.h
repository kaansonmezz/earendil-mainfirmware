#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H

#include "stm32h7xx_hal.h"

/* Stage 2 bring-up: minimal I2C bus scanner for MPU9250 detection.
 * Not the final MPU9250 driver — only verifies sensor visibility on I2C1. */

void I2C_ScanBus(void);

/* Probe a 7-bit I2C address using the caller's handle and the standard
 * 7-bit-to-HAL shift convention.  Returns HAL_OK if device ACKs. */
HAL_StatusTypeDef I2C_Scanner_Probe7(I2C_HandleTypeDef *hi2c,
                                     uint8_t addr7,
                                     uint32_t *err_out);

/* Probe a range of 7-bit addresses as a warm-up sequence.
 * Probes each address from start7 to end7 inclusive.
 * Prints only detected addresses. */
void I2C_Scanner_WarmupRange(I2C_HandleTypeDef *hi2c,
                             uint8_t start7,
                             uint8_t end7);

/* Probe from start7 up to target7 inclusive.  Stop immediately when
 * target7 returns HAL_OK.  Returns HAL_OK if target was found. */
HAL_StatusTypeDef I2C_Scanner_WarmupUntilFound(I2C_HandleTypeDef *hi2c,
                                               uint8_t start7,
                                               uint8_t target7,
                                               uint32_t *err_out);

#endif /* I2C_SCANNER_H */
