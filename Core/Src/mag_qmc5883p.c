#include "mag_qmc5883p.h"
#include "logger.h"
#include <string.h>

MAG_QMC5883P_Handle_t g_mag_handle = {0};

/* ── I2C timeout ─────────────────────────────────────────────────────────── */
#define MAG_I2C_TIMEOUT_MS  50U

/* ── Conversion constants for ±8 Gauss range ─────────────────────────────── */
#define MAG_QMC5883P_LSB_PER_GAUSS       3750
#define MAG_QMC5883P_UTX100_PER_GAUSS    10000  /* 100 µT/Gauss * 100 for x100 scaling */

/* ── Integer square root (Babylonian method) ──────────────────────────────── */
static uint32_t isqrt(uint32_t n)
{
    if (n == 0) return 0;
    uint32_t x = n;
    uint32_t y = (x + 1) / 2;
    while (y < x)
    {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

/* ── Convert raw to microtesla x100 ───────────────────────────────────────── */
static int32_t MAG_QMC5883P_RawToUtX100(int16_t raw)
{
    return ((int32_t)raw * MAG_QMC5883P_UTX100_PER_GAUSS) / MAG_QMC5883P_LSB_PER_GAUSS;
}

/* ── Low-level helpers ───────────────────────────────────────────────────── */

static HAL_StatusTypeDef Mag_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t addr7,
                                     uint8_t reg, uint8_t *value)
{
    uint16_t devAddr = (uint16_t)(addr7 << 1);
    return HAL_I2C_Mem_Read(hi2c, devAddr, reg, I2C_MEMADD_SIZE_8BIT,
                            value, 1, MAG_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Mag_ReadBytes(I2C_HandleTypeDef *hi2c, uint8_t addr7,
                                       uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t devAddr = (uint16_t)(addr7 << 1);
    return HAL_I2C_Mem_Read(hi2c, devAddr, reg, I2C_MEMADD_SIZE_8BIT,
                            buf, len, MAG_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef Mag_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t addr7,
                                      uint8_t reg, uint8_t value)
{
    uint16_t devAddr = (uint16_t)(addr7 << 1);
    return HAL_I2C_Mem_Write(hi2c, devAddr, reg, I2C_MEMADD_SIZE_8BIT,
                             &value, 1, MAG_I2C_TIMEOUT_MS);
}

/* ── Public register read ────────────────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    return Mag_ReadReg(hi2c, MAG_QMC5883P_ADDR7, reg, value);
}

/* ── Detection ───────────────────────────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_Detect(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag)
{
    if (mag == NULL)
        return HAL_ERROR;

    mag->found = 0;
    mag->initialized = 0;
    mag->addr7 = MAG_QMC5883P_ADDR7;
    mag->chip_id = 0;

    /* Probe address */
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(hi2c, MAG_QMC5883P_DEVADDR_HAL,
                                                  2, MAG_I2C_TIMEOUT_MS);
    if (st != HAL_OK)
        return HAL_ERROR;

    /* Read chip ID */
    uint8_t chip_id = 0;
    st = Mag_ReadReg(hi2c, MAG_QMC5883P_ADDR7, MAG_QMC5883P_REG_CHIP_ID, &chip_id);
    if (st != HAL_OK)
        return HAL_ERROR;

    mag->chip_id = chip_id;
    mag->found = 1;

    return HAL_OK;
}

/* ── Initialization ──────────────────────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_Init(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag)
{
    if (mag == NULL)
        return HAL_ERROR;

    /* Auto-detect if not already found */
    if (!mag->found)
    {
        HAL_StatusTypeDef st = MAG_QMC5883P_Detect(hi2c, mag);
        if (st != HAL_OK)
            return st;
    }

    /* Verify chip ID */
    if (mag->chip_id != MAG_QMC5883P_CHIP_ID_EXPECTED)
        return HAL_ERROR;

    HAL_StatusTypeDef st;

    /* Set/Reset period register (recommended 0x01 for QMC5883L) */
    st = Mag_WriteReg(hi2c, MAG_QMC5883P_ADDR7, MAG_QMC5883P_REG_SET_RESET, 0x01);
    if (st != HAL_OK)
        return st;

    /* CTRL2: default (0x00) */
    st = Mag_WriteReg(hi2c, MAG_QMC5883P_ADDR7, MAG_QMC5883P_REG_CTRL2, 0x00);
    if (st != HAL_OK)
        return st;

    /* CTRL1: Continuous mode, 200Hz ODR, 512 OSR, 8G range */
    st = Mag_WriteReg(hi2c, MAG_QMC5883P_ADDR7, MAG_QMC5883P_REG_CTRL1, 0xC5);
    if (st != HAL_OK)
        return st;

    mag->initialized = 1;
    return HAL_OK;
}

/* ── Raw read ────────────────────────────────────────────────────────────── */

#define MAG_DRDY_TIMEOUT_MS  10U

HAL_StatusTypeDef MAG_QMC5883P_ReadRaw(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag, MAG_QMC5883P_Raw_t *raw)
{
    if (mag == NULL || raw == NULL)
        return HAL_ERROR;

    /* Auto-init if not already initialized */
    if (!mag->initialized)
    {
        HAL_StatusTypeDef st = MAG_QMC5883P_Init(hi2c, mag);
        if (st != HAL_OK)
            return st;
    }

    memset(raw, 0, sizeof(*raw));
    raw->chip_id = mag->chip_id;

    /* Poll STATUS.DRDY before reading data */
    uint32_t t0 = HAL_GetTick();
    HAL_StatusTypeDef st;
    do
    {
        st = Mag_ReadReg(hi2c, MAG_QMC5883P_ADDR7,
                         MAG_QMC5883P_REG_STATUS, &raw->status);
        if (st != HAL_OK)
            return st;
        if (raw->status & MAG_QMC5883P_STATUS_DRDY)
            break;
    } while ((HAL_GetTick() - t0) < MAG_DRDY_TIMEOUT_MS);

    if (!(raw->status & MAG_QMC5883P_STATUS_DRDY))
        return HAL_TIMEOUT;

    /* Read 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB */
    uint8_t buf[6];
    st = Mag_ReadBytes(hi2c, MAG_QMC5883P_ADDR7, MAG_QMC5883P_REG_X_LSB, buf, 6);
    if (st != HAL_OK)
        return st;

    /* Decode little-endian signed 16-bit values */
    raw->x = (int16_t)((buf[1] << 8) | buf[0]);
    raw->y = (int16_t)((buf[3] << 8) | buf[2]);
    raw->z = (int16_t)((buf[5] << 8) | buf[4]);

    return HAL_OK;
}

/* ── MAG_IMU compact telemetry ────────────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_ReadImu(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag)
{
    MAG_QMC5883P_Raw_t raw;
    HAL_StatusTypeDef st = MAG_QMC5883P_ReadRaw(hi2c, mag, &raw);
    if (st != HAL_OK)
    {
        Logger_Log(LOG_INFO, "MAG_IMU,OK:0,ERR:READ_FAIL");
        return HAL_ERROR;
    }

    uint8_t drdy = (raw.status & MAG_QMC5883P_STATUS_DRDY) ? 1U : 0U;
    uint8_t ovfl = (raw.status & MAG_QMC5883P_STATUS_OVFL) ? 1U : 0U;

    /* Convert to microtesla x100 */
    int32_t mx_utx100 = MAG_QMC5883P_RawToUtX100(raw.x);
    int32_t my_utx100 = MAG_QMC5883P_RawToUtX100(raw.y);
    int32_t mz_utx100 = MAG_QMC5883P_RawToUtX100(raw.z);

    /* Calculate magnetic vector magnitude: sqrt(mx^2 + my^2 + mz^2) in µT x100 */
    int64_t sum_sq = (int64_t)mx_utx100 * mx_utx100 +
                     (int64_t)my_utx100 * my_utx100 +
                     (int64_t)mz_utx100 * mz_utx100;
    int32_t bmag_utx100 = (int32_t)isqrt((uint32_t)sum_sq);

    Logger_Log(LOG_INFO,
               "MAG_IMU,MX:%d,MY:%d,MZ:%d,"
               "MX_UTX100:%ld,MY_UTX100:%ld,MZ_UTX100:%ld,BMAG_UTX100:%ld,"
               "STATUS:0x%02X,DRDY:%u,OVFL:%u,OK:1",
               (int)raw.x, (int)raw.y, (int)raw.z,
               (long)mx_utx100, (long)my_utx100, (long)mz_utx100, (long)bmag_utx100,
               (unsigned)raw.status, drdy, ovfl);

    return HAL_OK;
}
