#ifndef MAG_QMC5883P_H
#define MAG_QMC5883P_H

#include "stm32h7xx_hal.h"

/* ── I2C address ─────────────────────────────────────────────────────────── */
#define MAG_QMC5883P_ADDR7              0x0DU
#define MAG_QMC5883P_DEVADDR_HAL        (MAG_QMC5883P_ADDR7 << 1)  /* 0x1A */

/* ── Register map (QMC5883L) ─────────────────────────────────────────────── */
#define MAG_QMC5883P_REG_CHIP_ID        0x0DU
#define MAG_QMC5883P_REG_X_LSB          0x00U
#define MAG_QMC5883P_REG_X_MSB          0x01U
#define MAG_QMC5883P_REG_Y_LSB          0x02U
#define MAG_QMC5883P_REG_Y_MSB          0x03U
#define MAG_QMC5883P_REG_Z_LSB          0x04U
#define MAG_QMC5883P_REG_Z_MSB          0x05U
#define MAG_QMC5883P_REG_STATUS         0x06U
#define MAG_QMC5883P_REG_CTRL1          0x09U
#define MAG_QMC5883P_REG_CTRL2          0x0AU
#define MAG_QMC5883P_REG_SET_RESET      0x0BU

#define MAG_QMC5883P_CHIP_ID_EXPECTED   0xFFU

/* ── Status register bits ────────────────────────────────────────────────── */
#define MAG_QMC5883P_STATUS_DRDY        (1U << 0)
#define MAG_QMC5883P_STATUS_OVFL        (1U << 1)

/* ── Raw sensor data ─────────────────────────────────────────────────────── */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t status;
    uint8_t chip_id;
} MAG_QMC5883P_Raw_t;

/* ── State handle ────────────────────────────────────────────────────────── */
typedef struct
{
    uint8_t found;
    uint8_t initialized;
    uint8_t addr7;
    uint8_t chip_id;
} MAG_QMC5883P_Handle_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

extern MAG_QMC5883P_Handle_t g_mag_handle;

/* Low-level register read (polling I2C). */
HAL_StatusTypeDef MAG_QMC5883P_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value);

/* Probe address 0x0D and read chip ID register 0x0D.
 * Populates handle on success. Does not write any registers. */
HAL_StatusTypeDef MAG_QMC5883P_Detect(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag);

/* Initialize QMC5883L: set/reset period, control registers.
 * Calls detect internally if not already found. */
HAL_StatusTypeDef MAG_QMC5883P_Init(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag);

/* Read raw magnetic X/Y/Z, status, and chip ID.
 * Calls detect internally if not already found. */
HAL_StatusTypeDef MAG_QMC5883P_ReadRaw(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag, MAG_QMC5883P_Raw_t *raw);

/* Read and print compact MAG_IMU telemetry line.
 * Calls detect internally if not already found. */
HAL_StatusTypeDef MAG_QMC5883P_ReadImu(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag);

#endif /* MAG_QMC5883P_H */
