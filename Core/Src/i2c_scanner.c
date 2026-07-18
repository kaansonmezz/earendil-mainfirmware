/* Stage 2 bring-up: minimal I2C bus scanner for MPU9250 detection.
 * Uses HAL_I2C_IsDeviceReady() polling on I2C1 (PB8=SCL, PB9=SDA).
 * Not the final MPU9250 driver — only verifies sensor visibility on I2C1. */

#include "i2c_scanner.h"
#include "i2c_recovery.h"
#include "logger.h"
#include "stm32h7xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define I2C_SCAN_ADDR_MIN   0x03U
#define I2C_SCAN_ADDR_MAX   0x77U
#define I2C_SCAN_TRIALS     2U
#define I2C_SCAN_TIMEOUT_MS 5U

HAL_StatusTypeDef I2C_Scanner_Probe7(I2C_HandleTypeDef *hi2c,
                                     uint8_t addr7,
                                     uint32_t *err_out)
{
    HAL_StatusTypeDef st;

    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    st = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr7 << 1),
                               I2C_SCAN_TRIALS, I2C_SCAN_TIMEOUT_MS);
    if (err_out != NULL)
        *err_out = HAL_I2C_GetError(hi2c);

    return st;
}

void I2C_ScanBus(void)
{
    Logger_Log(LOG_INFO, "I2C_SCAN,START");

    Logger_Log(LOG_INFO,
               "SCAN_I2C_HANDLE,PTR:0x%08lX,INSTANCE:0x%08lX,"
               "STATE:%lu,MODE:%lu,ERR:%lu",
               (unsigned long)(uintptr_t)&hi2c1,
               (unsigned long)(uintptr_t)hi2c1.Instance,
               (unsigned long)HAL_I2C_GetState(&hi2c1),
               (unsigned long)hi2c1.Mode,
               (unsigned long)hi2c1.ErrorCode);

    uint8_t count = 0U;

    for (uint8_t addr = I2C_SCAN_ADDR_MIN; addr <= I2C_SCAN_ADDR_MAX; addr++)
    {
        uint32_t err = 0;
        HAL_StatusTypeDef st = I2C_Scanner_Probe7(&hi2c1, addr, &err);

        if (st == HAL_OK)
        {
            Logger_Log(LOG_INFO, "I2C_SCAN,ADDR:0x%02X", addr);
            Logger_Log(LOG_INFO,
                       "I2C_SCAN,ADDR7:0x%02X,DEVADDR_HAL:0x%02X,HAL:%d,ERR:%lu",
                       addr, (unsigned)(addr << 1), (int)st, (unsigned long)err);
            count++;
        }
    }

    Logger_Log(LOG_INFO, "I2C_SCAN,DONE,COUNT:%u", count);

    /* If no devices found, bus may be stuck — try recovery and re-scan. */
    if (count == 0U)
    {
        Logger_Log(LOG_INFO, "I2C_SCAN,NO_DEVICES,TRYING_RECOVERY");
        I2C_BusRecovery(&hi2c1);

        count = 0U;
        for (uint8_t addr = I2C_SCAN_ADDR_MIN; addr <= I2C_SCAN_ADDR_MAX; addr++)
        {
            uint32_t err = 0;
            HAL_StatusTypeDef st2 = I2C_Scanner_Probe7(&hi2c1, addr, &err);
            if (st2 == HAL_OK)
            {
                Logger_Log(LOG_INFO, "I2C_SCAN,ADDR:0x%02X", addr);
                count++;
            }
        }
        Logger_Log(LOG_INFO, "I2C_SCAN,AFTER_RECOVERY,COUNT:%u", count);
    }
}

void I2C_Scanner_WarmupRange(I2C_HandleTypeDef *hi2c,
                             uint8_t start7,
                             uint8_t end7)
{
    uint8_t count = 0U;

    for (uint8_t addr = start7; addr <= end7; addr++)
    {
        uint32_t err = 0;
        HAL_StatusTypeDef st = I2C_Scanner_Probe7(hi2c, addr, &err);

        if (st == HAL_OK)
        {
            Logger_Log(LOG_INFO,
                       "I2C_WARMUP,ADDR7:0x%02X,DEVADDR_HAL:0x%02X,HAL:%d,ERR:%lu",
                       addr, (unsigned)(addr << 1), (int)st, (unsigned long)err);
            count++;
        }
    }

    Logger_Log(LOG_INFO, "I2C_WARMUP,DONE,START:0x%02X,END:0x%02X,COUNT:%u",
               start7, end7, count);
}

HAL_StatusTypeDef I2C_Scanner_WarmupUntilFound(I2C_HandleTypeDef *hi2c,
                                               uint8_t start7,
                                               uint8_t target7,
                                               uint32_t *err_out)
{
    for (uint8_t addr = start7; addr <= target7; addr++)
    {
        uint32_t err = 0;
        HAL_StatusTypeDef st = I2C_Scanner_Probe7(hi2c, addr, &err);

        if (st == HAL_OK)
        {
            Logger_Log(LOG_INFO,
                       "I2C_WARMUP_TARGET,ADDR7:0x%02X,DEVADDR_HAL:0x%02X,HAL:%d,ERR:%lu",
                       addr, (unsigned)(addr << 1), (int)st, (unsigned long)err);
        }

        if (addr == target7)
        {
            if (err_out != NULL)
                *err_out = err;
            return st;
        }
    }

    /* Should never reach here, but handle gracefully. */
    if (err_out != NULL)
        *err_out = 0;
    Logger_Log(LOG_INFO,
               "I2C_WARMUP_TARGET_FAIL,START:0x%02X,TARGET:0x%02X,HAL:-1,ERR:0",
               start7, target7);
    return HAL_ERROR;
}
