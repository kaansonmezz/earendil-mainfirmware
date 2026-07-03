/* Stage 3+4: MPU6500/MPU9250 WHO_AM_I probe and basic init.
 * Uses I2C1 polling.  No magnetometer, no continuous reading, no calibration. */

#include "imu_mpu9250.h"
#include "i2c_scanner.h"
#include "logger.h"

#define MPU_I2C_TIMEOUT_MS  50U

/* ── Supported WHO_AM_I helper ───────────────────────────────────────────── */

uint8_t IMU_MPU9250_IsSupportedWho(uint8_t who)
{
    return (who == 0x70U || who == 0x71U || who == 0x73U);
}

static const char *ChipName(uint8_t who)
{
    switch (who)
    {
        case 0x70U: return "MPU6500_COMPAT";
        case 0x71U: return "MPU9250_COMPAT";
        case 0x73U: return "MPU925X_ALT";
        default:    return "UNKNOWN";
    }
}

/* ── Low-level register helpers ──────────────────────────────────────────── */

HAL_StatusTypeDef IMU_MPU9250_WriteReg(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg, uint8_t value)
{
    /* Method A: HAL memory write */
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    HAL_StatusTypeDef mem_st = HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR_HAL, reg,
                                                  I2C_MEMADD_SIZE_8BIT,
                                                  &value, 1,
                                                  MPU_I2C_TIMEOUT_MS);
    uint32_t mem_err = HAL_I2C_GetError(hi2c);

    if (mem_st == HAL_OK)
    {
        Logger_Log(LOG_INFO,
                   "MPU_WRITE,REG:0x%02X,VAL:0x%02X,"
                   "MEM_HAL:%d,MEM_ERR:%lu,MAN_HAL:-,MAN_ERR:-,OK:1",
                   reg, value, (int)mem_st, (unsigned long)mem_err);
        return HAL_OK;
    }

    /* Method B: manual 2-byte write (register + value) */
    uint8_t buf[2] = { reg, value };
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    HAL_StatusTypeDef man_st = HAL_I2C_Master_Transmit(hi2c, MPU9250_ADDR_HAL,
                                                        buf, 2,
                                                        MPU_I2C_TIMEOUT_MS);
    uint32_t man_err = HAL_I2C_GetError(hi2c);

    uint8_t ok = (mem_st == HAL_OK || man_st == HAL_OK) ? 1U : 0U;
    Logger_Log(LOG_INFO,
               "MPU_WRITE,REG:0x%02X,VAL:0x%02X,"
               "MEM_HAL:%d,MEM_ERR:%lu,"
               "MAN_HAL:%d,MAN_ERR:%lu,OK:%u",
               reg, value,
               (int)mem_st, (unsigned long)mem_err,
               (int)man_st, (unsigned long)man_err, ok);

    return ok ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef IMU_MPU9250_ReadReg(I2C_HandleTypeDef *hi2c,
                                      uint8_t reg, uint8_t *value)
{
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    return HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR_HAL, reg,
                            I2C_MEMADD_SIZE_8BIT, value, 1,
                            MPU_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef IMU_MPU9250_FindAndWriteReg(I2C_HandleTypeDef *hi2c,
                                               uint8_t reg, uint8_t value)
{
    for (uint8_t addr = 0x03; addr <= 0x68; addr++)
    {
        uint32_t err = 0;
        HAL_StatusTypeDef probe = I2C_Scanner_Probe7(hi2c, addr, &err);

        if (addr < 0x68)
        {
            (void)probe;
            continue;
        }

        if (probe != HAL_OK)
        {
            Logger_Log(LOG_INFO,
                       "MPU_FINDWRITE_ABORT,REASON:TARGET_NOT_FOUND,"
                       "REG:0x%02X,VAL:0x%02X", reg, value);
            return HAL_ERROR;
        }

        /* Method A: HAL memory write */
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef mem_st = HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR_HAL,
                                                      reg,
                                                      I2C_MEMADD_SIZE_8BIT,
                                                      &value, 1, 100);
        uint32_t mem_err = HAL_I2C_GetError(hi2c);

        if (mem_st == HAL_OK)
        {
            Logger_Log(LOG_INFO,
                       "MPU_FINDWRITE,ADDR:0x68,REG:0x%02X,VAL:0x%02X,"
                       "MEM_HAL:%d,MEM_ERR:%lu,MAN_HAL:-,MAN_ERR:-,OK:1",
                       reg, value, (int)mem_st, (unsigned long)mem_err);
            return HAL_OK;
        }

        /* Method B: manual 2-byte write */
        uint8_t buf[2] = { reg, value };
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef man_st = HAL_I2C_Master_Transmit(hi2c,
                                                            MPU9250_ADDR_HAL,
                                                            buf, 2, 100);
        uint32_t man_err = HAL_I2C_GetError(hi2c);

        uint8_t ok = (mem_st == HAL_OK || man_st == HAL_OK) ? 1U : 0U;
        Logger_Log(LOG_INFO,
                   "MPU_FINDWRITE,ADDR:0x68,REG:0x%02X,VAL:0x%02X,"
                   "MEM_HAL:%d,MEM_ERR:%lu,"
                   "MAN_HAL:%d,MAN_ERR:%lu,OK:%u",
                   reg, value,
                   (int)mem_st, (unsigned long)mem_err,
                   (int)man_st, (unsigned long)man_err, ok);

        return ok ? HAL_OK : HAL_ERROR;
    }

    Logger_Log(LOG_INFO,
               "MPU_FINDWRITE_ABORT,REASON:TARGET_NOT_FOUND,"
               "REG:0x%02X,VAL:0x%02X", reg, value);
    return HAL_ERROR;
}

HAL_StatusTypeDef IMU_MPU9250_FindAndReadReg(I2C_HandleTypeDef *hi2c,
                                              uint8_t reg, uint8_t *value)
{
    for (uint8_t addr = 0x03; addr <= 0x68; addr++)
    {
        uint32_t err = 0;
        HAL_StatusTypeDef probe = I2C_Scanner_Probe7(hi2c, addr, &err);

        if (addr < 0x68)
        {
            (void)probe;
            continue;
        }

        if (probe != HAL_OK)
        {
            Logger_Log(LOG_INFO,
                       "MPU_FINDREAD_ABORT,REASON:TARGET_NOT_FOUND,"
                       "REG:0x%02X", reg);
            return HAL_ERROR;
        }

        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef mem_st = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR_HAL,
                                                     reg,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     value, 1, 100);
        uint32_t mem_err = HAL_I2C_GetError(hi2c);

        Logger_Log(LOG_INFO,
                   "MPU_FINDREAD,ADDR:0x68,REG:0x%02X,"
                   "MEM_HAL:%d,MEM_ERR:%lu,VAL:0x%02X,OK:%u",
                   reg, (int)mem_st, (unsigned long)mem_err,
                   *value, (mem_st == HAL_OK) ? 1U : 0U);

        return mem_st;
    }

    Logger_Log(LOG_INFO,
               "MPU_FINDREAD_ABORT,REASON:TARGET_NOT_FOUND,"
               "REG:0x%02X", reg);
    return HAL_ERROR;
}

/* ── Stage 3: WHO_AM_I probe (unchanged logic, updated OK rule) ─────────── */

HAL_StatusTypeDef IMU_MPU9250_FindAndReadWho(I2C_HandleTypeDef *hi2c,
                                              uint8_t *who_out)
{
    uint8_t who_mem = 0xEE;
    uint8_t who_manual = 0xEE;
    uint8_t reg = MPU9250_REG_WHO_AM_I;

    for (uint8_t addr = 0x03; addr <= 0x68; addr++)
    {
        uint32_t err = 0;
        HAL_StatusTypeDef probe = I2C_Scanner_Probe7(hi2c, addr, &err);

        if (addr < 0x68)
        {
            (void)probe;
            continue;
        }

        if (probe != HAL_OK)
        {
            Logger_Log(LOG_INFO,
                       "MPU_FINDWHO,ADDR:0x68,MEM_HAL:-1,MEM_ERR:0,MEM_WHO:0xEE,"
                       "TX_HAL:-1,TX_ERR:0,RX_HAL:-1,RX_ERR:0,MAN_WHO:0xEE,OK:0");
            return HAL_ERROR;
        }

        /* Mem_Read */
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef mem = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR_HAL,
                                                  MPU9250_REG_WHO_AM_I,
                                                  I2C_MEMADD_SIZE_8BIT,
                                                  &who_mem, 1, 100);
        uint32_t mem_err = HAL_I2C_GetError(hi2c);

        /* Manual TX + RX */
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef tx = HAL_I2C_Master_Transmit(hi2c, MPU9250_ADDR_HAL,
                                                        &reg, 1, 100);
        uint32_t tx_err = HAL_I2C_GetError(hi2c);

        if (tx == HAL_OK)
            HAL_Delay(1);

        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef rx = HAL_ERROR;
        uint32_t rx_err = 0;
        if (tx == HAL_OK)
        {
            rx = HAL_I2C_Master_Receive(hi2c, MPU9250_ADDR_HAL,
                                         &who_manual, 1, 100);
            rx_err = HAL_I2C_GetError(hi2c);
        }

        uint8_t ok = ((mem == HAL_OK && IMU_MPU9250_IsSupportedWho(who_mem)) ||
                      (rx == HAL_OK && IMU_MPU9250_IsSupportedWho(who_manual)))
                     ? 1U : 0U;

        if (mem == HAL_OK && IMU_MPU9250_IsSupportedWho(who_mem))
            *who_out = who_mem;
        else if (rx == HAL_OK && IMU_MPU9250_IsSupportedWho(who_manual))
            *who_out = who_manual;
        else
            *who_out = (who_mem != 0xEE) ? who_mem : who_manual;

        Logger_Log(LOG_INFO,
                   "MPU_FINDWHO,ADDR:0x68,"
                   "MEM_HAL:%d,MEM_ERR:%lu,MEM_WHO:0x%02X,"
                   "TX_HAL:%d,TX_ERR:%lu,"
                   "RX_HAL:%d,RX_ERR:%lu,"
                   "MAN_WHO:0x%02X,OK:%u",
                   (int)mem, (unsigned long)mem_err, who_mem,
                   (int)tx, (unsigned long)tx_err,
                   (int)rx, (unsigned long)rx_err,
                   who_manual, ok);

        return ok ? HAL_OK : HAL_ERROR;
    }

    Logger_Log(LOG_INFO,
               "MPU_FINDWHO,ADDR:0x68,MEM_HAL:-1,MEM_ERR:0,MEM_WHO:0xEE,"
               "TX_HAL:-1,TX_ERR:0,RX_HAL:-1,RX_ERR:0,MAN_WHO:0xEE,OK:0");
    return HAL_ERROR;
}

void IMU_MPU9250_WhoAmI(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0xEE;
    HAL_StatusTypeDef st = IMU_MPU9250_FindAndReadWho(hi2c, &who);

    if (st == HAL_OK && IMU_MPU9250_IsSupportedWho(who))
        Logger_Log(LOG_INFO, "MPU_WHO_FINAL,WHO:0x%02X,CHIP:%s,OK:1",
                   who, ChipName(who));
    else
        Logger_Log(LOG_INFO, "MPU_WHO_FINAL,WHO:0x%02X,CHIP:%s,OK:0",
                   who, ChipName(who));
}

void IMU_MPU9250_WarmupProbe(I2C_HandleTypeDef *hi2c)
{
    uint32_t err;

    Logger_Log(LOG_INFO,
               "MPU_I2C_HANDLE,PTR:0x%08lX,INSTANCE:0x%08lX,"
               "STATE:%lu,MODE:%lu,ERR:%lu",
               (unsigned long)(uintptr_t)hi2c,
               (unsigned long)(uintptr_t)hi2c->Instance,
               (unsigned long)HAL_I2C_GetState(hi2c),
               (unsigned long)hi2c->Mode,
               (unsigned long)hi2c->ErrorCode);

    err = 0;
    HAL_StatusTypeDef probe_before = I2C_Scanner_Probe7(hi2c, 0x68, &err);

    Logger_Log(LOG_INFO,
               "MPU_PROBE_DIRECT_BEFORE,ADDR7:0x68,DEVADDR_HAL:0xD0,"
               "HAL:%d,ERR:%lu,STATE:%lu",
               (int)probe_before, (unsigned long)err,
               (unsigned long)HAL_I2C_GetState(hi2c));

    err = 0;
    HAL_StatusTypeDef warmup = I2C_Scanner_WarmupUntilFound(hi2c, 0x03, 0x68, &err);

    Logger_Log(LOG_INFO,
               "MPU_WARM_TARGET_RESULT,ADDR:0x68,HAL:%d,ERR:%lu",
               (int)warmup, (unsigned long)err);
}

/* ── Stage 4: basic init ─────────────────────────────────────────────────── */

static HAL_StatusTypeDef WriteAndVerify(I2C_HandleTypeDef *hi2c,
                                        uint8_t reg, uint8_t val,
                                        const char *step_name)
{
    HAL_StatusTypeDef write_st = IMU_MPU9250_FindAndWriteReg(hi2c, reg, val);
    uint8_t write_ok = (write_st == HAL_OK) ? 1U : 0U;

    if (!write_ok)
    {
        Logger_Log(LOG_INFO, "MPU_INIT,DONE,OK:0,FAIL_AT:%s", step_name);
        return HAL_ERROR;
    }

    /* Readback verification using find-and-read path */
    uint8_t readback = 0;
    HAL_StatusTypeDef read_st = IMU_MPU9250_FindAndReadReg(hi2c, reg, &readback);
    uint8_t read_ok = (read_st == HAL_OK) ? 1U : 0U;
    uint8_t match = (readback == val) ? 1U : 0U;
    uint8_t verify_ok = (write_ok && read_ok && match) ? 1U : 0U;

    Logger_Log(LOG_INFO,
               "MPU_WRITE_VERIFY,REG:0x%02X,VAL:0x%02X,"
               "READ_VAL:0x%02X,OK:%u",
               reg, val, readback, verify_ok);

    if (!verify_ok)
    {
        Logger_Log(LOG_INFO, "MPU_INIT,DONE,OK:0,FAIL_AT:%s", step_name);
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef WriteAndVerifyOptional(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg, uint8_t val,
                                                const char *step_name)
{
    HAL_StatusTypeDef write_st = IMU_MPU9250_FindAndWriteReg(hi2c, reg, val);
    uint8_t write_ok = (write_st == HAL_OK) ? 1U : 0U;

    /* Readback verification */
    uint8_t readback = 0;
    HAL_StatusTypeDef read_st = IMU_MPU9250_FindAndReadReg(hi2c, reg, &readback);
    uint8_t read_ok = (read_st == HAL_OK) ? 1U : 0U;
    uint8_t match = (readback == val) ? 1U : 0U;
    uint8_t verify_ok = (write_ok && read_ok && match) ? 1U : 0U;

    Logger_Log(LOG_INFO,
               "MPU_WRITE_VERIFY,REG:0x%02X,VAL:0x%02X,"
               "READ_VAL:0x%02X,OK:%u",
               reg, val, readback, verify_ok);

    if (!verify_ok)
    {
        Logger_Log(LOG_INFO,
                   "MPU_INIT_WARN,REG:0x%02X,EXPECTED:0x%02X,"
                   "READ_VAL:0x%02X,ACTION:USING_DEFAULT",
                   reg, val, readback);
    }

    return HAL_OK;
}

HAL_StatusTypeDef IMU_MPU9250_InitBasic(I2C_HandleTypeDef *hi2c)
{
    Logger_Log(LOG_INFO, "MPU_INIT,START,ADDR:0x%02X", MPU9250_ADDR_7BIT);

    /* a. Read WHO_AM_I and verify */
    uint8_t who = 0xEE;
    HAL_StatusTypeDef st = IMU_MPU9250_FindAndReadWho(hi2c, &who);
    if (st != HAL_OK || !IMU_MPU9250_IsSupportedWho(who))
    {
        Logger_Log(LOG_INFO, "MPU_INIT,WHO:0x%02X,CHIP:%s,OK:0", who, ChipName(who));
        Logger_Log(LOG_INFO, "MPU_INIT,DONE,OK:0,FAIL_AT:WHO_CHECK");
        return HAL_ERROR;
    }
    Logger_Log(LOG_INFO, "MPU_INIT,WHO:0x%02X,CHIP:%s,OK:1", who, ChipName(who));
    uint8_t is_compat_mode = (who == 0x70U) ? 1U : 0U;

    /* b. Reset device using find-and-write path */
    HAL_StatusTypeDef reset_st = IMU_MPU9250_FindAndWriteReg(hi2c,
                                                              MPU9250_REG_PWR_MGMT_1,
                                                              0x80);
    uint8_t reset_write_ok = (reset_st == HAL_OK) ? 1U : 0U;
    HAL_Delay(100);

    /* c. Verify WHO after reset */
    uint8_t who_after = 0xEE;
    HAL_StatusTypeDef who_st = IMU_MPU9250_FindAndReadWho(hi2c, &who_after);
    uint8_t who_ok = (who_st == HAL_OK && IMU_MPU9250_IsSupportedWho(who_after))
                     ? 1U : 0U;
    Logger_Log(LOG_INFO, "MPU_RESET_VERIFY,WRITE_OK:%u,WHO:0x%02X,WHO_OK:%u",
               reset_write_ok, who_after, who_ok);

    if (!reset_write_ok)
    {
        Logger_Log(LOG_INFO, "MPU_INIT,DONE,OK:0,FAIL_AT:RESET");
        return HAL_ERROR;
    }

    if (!who_ok)
    {
        Logger_Log(LOG_INFO, "MPU_INIT,DONE,OK:0,FAIL_AT:RESET_VERIFY");
        return HAL_ERROR;
    }

    /* d. Wake device, select clock source (REQUIRED) */
    if (WriteAndVerify(hi2c, MPU9250_REG_PWR_MGMT_1, 0x00, "WAKE") != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    /* e. Enable all accel/gyro axes (REQUIRED) */
    if (WriteAndVerify(hi2c, MPU9250_REG_PWR_MGMT_2, 0x00, "PWR_MGMT_2") != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    /* f. DLPF config (OPTIONAL for WHO=0x70) */
    if (is_compat_mode)
        WriteAndVerifyOptional(hi2c, MPU9250_REG_CONFIG, 0x03, "CONFIG");
    else
        if (WriteAndVerify(hi2c, MPU9250_REG_CONFIG, 0x03, "CONFIG") != HAL_OK)
            return HAL_ERROR;

    /* g. Sample rate divider (OPTIONAL for WHO=0x70) */
    if (is_compat_mode)
        WriteAndVerifyOptional(hi2c, MPU9250_REG_SMPLRT_DIV, 0x09, "SMPLRT_DIV");
    else
        if (WriteAndVerify(hi2c, MPU9250_REG_SMPLRT_DIV, 0x09, "SMPLRT_DIV") != HAL_OK)
            return HAL_ERROR;

    /* h. Gyro range ±250 dps (REQUIRED, default 0x00) */
    if (WriteAndVerify(hi2c, MPU9250_REG_GYRO_CONFIG, 0x00, "GYRO_CONFIG") != HAL_OK)
        return HAL_ERROR;

    /* i. Accel range ±2g (REQUIRED, default 0x00) */
    if (WriteAndVerify(hi2c, MPU9250_REG_ACCEL_CONFIG, 0x00, "ACCEL_CONFIG") != HAL_OK)
        return HAL_ERROR;

    /* j. Accel DLPF (OPTIONAL for WHO=0x70) */
    if (is_compat_mode)
        WriteAndVerifyOptional(hi2c, MPU9250_REG_ACCEL_CONFIG2, 0x03, "ACCEL_CONFIG2");
    else
        if (WriteAndVerify(hi2c, MPU9250_REG_ACCEL_CONFIG2, 0x03, "ACCEL_CONFIG2") != HAL_OK)
            return HAL_ERROR;

    /* k. Effective config summary: read back all config registers */
    uint8_t eff_pwr1 = 0, eff_pwr2 = 0, eff_cfg = 0, eff_smplrt = 0;
    uint8_t eff_gyro_cfg = 0, eff_accel_cfg = 0, eff_accel_cfg2 = 0;
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_PWR_MGMT_1, &eff_pwr1);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_PWR_MGMT_2, &eff_pwr2);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG, &eff_cfg);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_SMPLRT_DIV, &eff_smplrt);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_GYRO_CONFIG, &eff_gyro_cfg);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_ACCEL_CONFIG, &eff_accel_cfg);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_ACCEL_CONFIG2, &eff_accel_cfg2);

    Logger_Log(LOG_INFO,
               "MPU_INIT_EFFECTIVE,PWR1:0x%02X,PWR2:0x%02X,"
               "CONFIG:0x%02X,SMPLRT:0x%02X,"
               "GYRO_CFG:0x%02X,ACCEL_CFG:0x%02X,ACCEL_CFG2:0x%02X",
               eff_pwr1, eff_pwr2, eff_cfg, eff_smplrt,
               eff_gyro_cfg, eff_accel_cfg, eff_accel_cfg2);

    if (is_compat_mode)
        Logger_Log(LOG_INFO, "MPU_INIT,DONE,OK:1,MODE:MINIMAL_COMPAT");
    else
        Logger_Log(LOG_INFO, "MPU_INIT,DONE,OK:1");

    return HAL_OK;
}

/* ── Diagnostic: CONFIG register write/readback test ──────────────────────── */

void IMU_MPU9250_CfgTest(I2C_HandleTypeDef *hi2c)
{
    Logger_Log(LOG_INFO, "MPU_CFGTEST,START");

    /* a. Read WHO_AM_I first */
    uint8_t who = 0xEE;
    HAL_StatusTypeDef who_st = IMU_MPU9250_FindAndReadWho(hi2c, &who);
    uint8_t who_ok = (who_st == HAL_OK && IMU_MPU9250_IsSupportedWho(who)) ? 1U : 0U;
    Logger_Log(LOG_INFO, "MPU_CFGTEST,WHO:0x%02X,OK:%u", who, who_ok);

    if (!who_ok)
    {
        Logger_Log(LOG_INFO, "MPU_CFGTEST,ABORT,REASON:WHO_FAIL");
        return;
    }

    /* b. Wake the chip: write PWR_MGMT_1 = 0x01, delay 20ms, read back */
    HAL_StatusTypeDef wake_st = IMU_MPU9250_FindAndWriteReg(hi2c, MPU9250_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(20);
    uint8_t pwr1 = 0;
    HAL_StatusTypeDef pwr1_st = IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_PWR_MGMT_1, &pwr1);
    uint8_t pwr_ok = (wake_st == HAL_OK && pwr1_st == HAL_OK && pwr1 == 0x01) ? 1U : 0U;
    Logger_Log(LOG_INFO, "MPU_CFGTEST,PWR_MGMT_1:0x%02X,OK:%u", pwr1, pwr_ok);

    if (!pwr_ok)
    {
        Logger_Log(LOG_INFO, "MPU_CFGTEST,ABORT,REASON:WAKE_FAIL");
        return;
    }

    /* c. Read initial CONFIG register */
    uint8_t cfg_before = 0;
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG, &cfg_before);
    Logger_Log(LOG_INFO, "MPU_CFGTEST,REG:0x1A,BEFORE:0x%02X", cfg_before);

    /* d. Test values: 0x00, 0x01, 0x03, 0x06 */
    static const uint8_t test_vals[] = { 0x00, 0x01, 0x03, 0x06 };

    for (size_t i = 0; i < sizeof(test_vals); i++)
    {
        uint8_t val = test_vals[i];

        /* ── Method A: HAL_I2C_Mem_Write ───────────────────────────── */
        {
            /* Find to 0x68 then immediate Mem_Write */
            HAL_StatusTypeDef probe = HAL_ERROR;
            for (uint8_t addr = 0x03; addr <= 0x68; addr++)
            {
                uint32_t err = 0;
                probe = I2C_Scanner_Probe7(hi2c, addr, &err);
                if (addr < 0x68)
                    continue;
                if (probe != HAL_OK)
                    break;

                /* Immediate Mem_Write */
                hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
                HAL_StatusTypeDef mem_st = HAL_I2C_Mem_Write(hi2c, MPU9250_ADDR_HAL,
                                                              MPU9250_REG_CONFIG,
                                                              I2C_MEMADD_SIZE_8BIT,
                                                              &val, 1, 100);
                uint32_t mem_err = HAL_I2C_GetError(hi2c);
                HAL_Delay(10);

                /* Read back three times */
                uint8_t r1 = 0, r2 = 0, r3 = 0;
                IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG, &r1);
                IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG, &r2);
                IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG, &r3);

                uint8_t ok = (mem_st == HAL_OK &&
                              (r1 == val || r2 == val || r3 == val)) ? 1U : 0U;
                Logger_Log(LOG_INFO,
                           "MPU_CFGTEST_MEM,VAL:0x%02X,HAL:%d,ERR:%lu,"
                           "R1:0x%02X,R2:0x%02X,R3:0x%02X,OK:%u",
                           val, (int)mem_st, (unsigned long)mem_err,
                           r1, r2, r3, ok);
                break;
            }
            if (probe != HAL_OK)
                Logger_Log(LOG_INFO, "MPU_CFGTEST_MEM,VAL:0x%02X,ABORT,REASON:PROBE_FAIL", val);
        }

        /* ── Method B: Manual 2-byte Master_Transmit ───────────────── */
        {
            HAL_StatusTypeDef probe = HAL_ERROR;
            for (uint8_t addr = 0x03; addr <= 0x68; addr++)
            {
                uint32_t err = 0;
                probe = I2C_Scanner_Probe7(hi2c, addr, &err);
                if (addr < 0x68)
                    continue;
                if (probe != HAL_OK)
                    break;

                /* Immediate manual write */
                uint8_t buf[2] = { MPU9250_REG_CONFIG, val };
                hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
                HAL_StatusTypeDef man_st = HAL_I2C_Master_Transmit(hi2c, MPU9250_ADDR_HAL,
                                                                    buf, 2, 100);
                uint32_t man_err = HAL_I2C_GetError(hi2c);
                HAL_Delay(10);

                /* Read back three times */
                uint8_t r1 = 0, r2 = 0, r3 = 0;
                IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG, &r1);
                IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG, &r2);
                IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG, &r3);

                uint8_t ok = (man_st == HAL_OK &&
                              (r1 == val || r2 == val || r3 == val)) ? 1U : 0U;
                Logger_Log(LOG_INFO,
                           "MPU_CFGTEST_MAN,VAL:0x%02X,HAL:%d,ERR:%lu,"
                           "R1:0x%02X,R2:0x%02X,R3:0x%02X,OK:%u",
                           val, (int)man_st, (unsigned long)man_err,
                           r1, r2, r3, ok);
                break;
            }
            if (probe != HAL_OK)
                Logger_Log(LOG_INFO, "MPU_CFGTEST_MAN,VAL:0x%02X,ABORT,REASON:PROBE_FAIL", val);
        }
    }

    /* Restore CONFIG to 0x03 (default DLPF) */
    IMU_MPU9250_FindAndWriteReg(hi2c, MPU9250_REG_CONFIG, 0x03);

    Logger_Log(LOG_INFO, "MPU_CFGTEST,DONE");
}

/* ── CubeIDE Live Expression debug variables ─────────────────────────────── */

/*
 * CubeIDE Live Expressions:
 * imu_dbg_acc_x
 * imu_dbg_acc_y
 * imu_dbg_acc_z
 * imu_dbg_temp
 * imu_dbg_gyro_x
 * imu_dbg_gyro_y
 * imu_dbg_gyro_z
 * imu_dbg_raw_ok
 * imu_dbg_raw_counter
 * imu_dbg_raw_fail_counter
 *
 * These are global (not static) so the debugger symbol table exposes them
 * reliably to CubeIDE Live Expressions.  __attribute__((used)) prevents
 * the linker from stripping them even if no code reads them directly.
 */
volatile int16_t imu_dbg_acc_x  __attribute__((used)) = 0;
volatile int16_t imu_dbg_acc_y  __attribute__((used)) = 0;
volatile int16_t imu_dbg_acc_z  __attribute__((used)) = 0;

volatile int16_t imu_dbg_temp   __attribute__((used)) = 0;

volatile int16_t imu_dbg_gyro_x __attribute__((used)) = 0;
volatile int16_t imu_dbg_gyro_y __attribute__((used)) = 0;
volatile int16_t imu_dbg_gyro_z __attribute__((used)) = 0;

volatile uint8_t  imu_dbg_raw_ok          __attribute__((used)) = 0;
volatile uint32_t imu_dbg_raw_counter     __attribute__((used)) = 0;
volatile uint32_t imu_dbg_raw_fail_counter __attribute__((used)) = 0;

void IMU_MPU9250_UpdateDebugRaw(const IMU_MPU9250_Raw_t *raw, uint8_t ok)
{
    if (raw != NULL && ok == 1)
    {
        imu_dbg_acc_x  = raw->acc_x;
        imu_dbg_acc_y  = raw->acc_y;
        imu_dbg_acc_z  = raw->acc_z;
        imu_dbg_temp   = raw->temp;
        imu_dbg_gyro_x = raw->gyro_x;
        imu_dbg_gyro_y = raw->gyro_y;
        imu_dbg_gyro_z = raw->gyro_z;
        imu_dbg_raw_ok = 1;
        imu_dbg_raw_counter++;
    }
    else
    {
        imu_dbg_raw_ok = 0;
        imu_dbg_raw_fail_counter++;
    }
}

/* ── Stage 5: burst read and raw accel/gyro ──────────────────────────────── */

HAL_StatusTypeDef IMU_MPU9250_FindAndReadBytes(I2C_HandleTypeDef *hi2c,
                                               uint8_t start_reg,
                                               uint8_t *buf, uint16_t len)
{
    for (uint8_t addr = 0x03; addr <= 0x68; addr++)
    {
        uint32_t err = 0;
        HAL_StatusTypeDef probe = I2C_Scanner_Probe7(hi2c, addr, &err);

        if (addr < 0x68)
        {
            (void)probe;
            continue;
        }

        if (probe != HAL_OK)
            return HAL_ERROR;

        /* Method A: HAL_I2C_Mem_Read burst */
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef mem_st = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR_HAL,
                                                     start_reg,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     buf, len, 100);

        if (mem_st == HAL_OK)
            return HAL_OK;

        /* Method B: manual register pointer write + Master_Receive */
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef tx_st = HAL_I2C_Master_Transmit(hi2c, MPU9250_ADDR_HAL,
                                                           &start_reg, 1, 100);

        HAL_StatusTypeDef rx_st = HAL_ERROR;
        if (tx_st == HAL_OK)
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
            rx_st = HAL_I2C_Master_Receive(hi2c, MPU9250_ADDR_HAL,
                                            buf, len, 100);
        }

        uint8_t ok = (mem_st == HAL_OK || rx_st == HAL_OK) ? 1U : 0U;
        return ok ? HAL_OK : HAL_ERROR;
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef IMU_MPU9250_FindAndReadBytesVerbose(I2C_HandleTypeDef *hi2c,
                                                      uint8_t start_reg,
                                                      uint8_t *buf, uint16_t len)
{
    for (uint8_t addr = 0x03; addr <= 0x68; addr++)
    {
        uint32_t err = 0;
        HAL_StatusTypeDef probe = I2C_Scanner_Probe7(hi2c, addr, &err);

        if (addr < 0x68)
        {
            (void)probe;
            continue;
        }

        if (probe != HAL_OK)
        {
            Logger_Log(LOG_INFO,
                       "MPU_FINDBURST_ABORT,REASON:TARGET_NOT_FOUND,"
                       "REG:0x%02X,LEN:%u", start_reg, len);
            return HAL_ERROR;
        }

        /* Method A: HAL_I2C_Mem_Read burst */
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef mem_st = HAL_I2C_Mem_Read(hi2c, MPU9250_ADDR_HAL,
                                                     start_reg,
                                                     I2C_MEMADD_SIZE_8BIT,
                                                     buf, len, 100);
        uint32_t mem_err = HAL_I2C_GetError(hi2c);

        if (mem_st == HAL_OK)
        {
            Logger_Log(LOG_INFO,
                       "MPU_FINDBURST,ADDR:0x68,REG:0x%02X,LEN:%u,"
                       "MEM_HAL:%d,MEM_ERR:%lu,TX_HAL:-,TX_ERR:-,"
                       "RX_HAL:-,RX_ERR:-,OK:1",
                       start_reg, len,
                       (int)mem_st, (unsigned long)mem_err);
            return HAL_OK;
        }

        /* Method B: manual register pointer write + Master_Receive */
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        HAL_StatusTypeDef tx_st = HAL_I2C_Master_Transmit(hi2c, MPU9250_ADDR_HAL,
                                                           &start_reg, 1, 100);
        uint32_t tx_err = HAL_I2C_GetError(hi2c);

        HAL_StatusTypeDef rx_st = HAL_ERROR;
        uint32_t rx_err = 0;
        if (tx_st == HAL_OK)
        {
            hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
            rx_st = HAL_I2C_Master_Receive(hi2c, MPU9250_ADDR_HAL,
                                            buf, len, 100);
            rx_err = HAL_I2C_GetError(hi2c);
        }

        uint8_t ok = (mem_st == HAL_OK || rx_st == HAL_OK) ? 1U : 0U;
        Logger_Log(LOG_INFO,
                   "MPU_FINDBURST,ADDR:0x68,REG:0x%02X,LEN:%u,"
                   "MEM_HAL:%d,MEM_ERR:%lu,"
                   "TX_HAL:%d,TX_ERR:%lu,"
                   "RX_HAL:%d,RX_ERR:%lu,OK:%u",
                   start_reg, len,
                   (int)mem_st, (unsigned long)mem_err,
                   (int)tx_st, (unsigned long)tx_err,
                   (int)rx_st, (unsigned long)rx_err, ok);

        return ok ? HAL_OK : HAL_ERROR;
    }

    Logger_Log(LOG_INFO,
               "MPU_FINDBURST_ABORT,REASON:TARGET_NOT_FOUND,"
               "REG:0x%02X,LEN:%u", start_reg, len);
    return HAL_ERROR;
}

HAL_StatusTypeDef IMU_MPU9250_ReadRaw(I2C_HandleTypeDef *hi2c,
                                      IMU_MPU9250_Raw_t *raw)
{
    uint8_t buf[14];
    HAL_StatusTypeDef st = IMU_MPU9250_FindAndReadBytes(hi2c,
                                                        MPU9250_REG_ACCEL_XOUT_H,
                                                        buf, 14);
    if (st != HAL_OK)
        return HAL_ERROR;

    raw->acc_x  = (int16_t)((buf[0]  << 8) | buf[1]);
    raw->acc_y  = (int16_t)((buf[2]  << 8) | buf[3]);
    raw->acc_z  = (int16_t)((buf[4]  << 8) | buf[5]);
    raw->temp   = (int16_t)((buf[6]  << 8) | buf[7]);
    raw->gyro_x = (int16_t)((buf[8]  << 8) | buf[9]);
    raw->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    raw->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);

    return HAL_OK;
}

/* ── Stage 5: gyro-specific diagnostic ───────────────────────────────────── */

void IMU_MPU9250_GyroTest(I2C_HandleTypeDef *hi2c)
{
    Logger_Log(LOG_INFO, "MPU_GYROTEST,START");

    /* ── Step 1: Read key configuration registers ────────────────────── */
    uint8_t who = 0, pwr1 = 0, pwr2 = 0, int_st = 0;
    uint8_t gyro_cfg = 0, accel_cfg = 0, cfg = 0, smplrt = 0;

    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_WHO_AM_I,       &who);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_PWR_MGMT_1,     &pwr1);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_PWR_MGMT_2,     &pwr2);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_INT_STATUS,     &int_st);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_GYRO_CONFIG,    &gyro_cfg);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_ACCEL_CONFIG,   &accel_cfg);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_CONFIG,         &cfg);
    IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_SMPLRT_DIV,     &smplrt);

    Logger_Log(LOG_INFO,
               "MPU_GYROTEST_REGS,"
               "WHO:0x%02X,PWR1:0x%02X,PWR2:0x%02X,INT:0x%02X,"
               "GYRO_CFG:0x%02X,ACCEL_CFG:0x%02X,CONFIG:0x%02X,SMPLRT:0x%02X",
               who, pwr1, pwr2, int_st, gyro_cfg, accel_cfg, cfg, smplrt);

    /* ── Step 2: Force gyro enabled (PWR_MGMT_2 = 0x00) ─────────────── */
    HAL_StatusTypeDef wr_st = IMU_MPU9250_FindAndWriteReg(hi2c,
                                                          MPU9250_REG_PWR_MGMT_2,
                                                          0x00);
    HAL_Delay(20);
    uint8_t pwr2_rb = 0;
    HAL_StatusTypeDef rd_st = IMU_MPU9250_FindAndReadReg(hi2c,
                                                         MPU9250_REG_PWR_MGMT_2,
                                                         &pwr2_rb);
    uint8_t pwr2_ok = (wr_st == HAL_OK && rd_st == HAL_OK && pwr2_rb == 0x00)
                      ? 1U : 0U;
    Logger_Log(LOG_INFO,
               "MPU_GYROTEST_ENABLE,"
               "PWR2_WRITE_OK:%u,PWR2_READ:0x%02X,OK:%u",
               (wr_st == HAL_OK) ? 1U : 0U, pwr2_rb, pwr2_ok);

    /* ── Step 3: Test different PWR_MGMT_1 clock sources ─────────────── */
    static const uint8_t clk_vals[] = { 0x00, 0x01, 0x02, 0x03 };
    for (size_t i = 0; i < sizeof(clk_vals); i++)
    {
        uint8_t val = clk_vals[i];
        IMU_MPU9250_FindAndWriteReg(hi2c, MPU9250_REG_PWR_MGMT_1, val);
        HAL_Delay(50);

        uint8_t pwr1_read = 0;
        IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_PWR_MGMT_1, &pwr1_read);

        uint8_t gbuf[6];
        HAL_StatusTypeDef gst = IMU_MPU9250_FindAndReadBytesVerbose(hi2c,
                                                              MPU9250_REG_GYRO_XOUT_H,
                                                              gbuf, 6);
        int16_t gx = 0, gy = 0, gz = 0;
        if (gst == HAL_OK)
        {
            gx = (int16_t)((gbuf[0] << 8) | gbuf[1]);
            gy = (int16_t)((gbuf[2] << 8) | gbuf[3]);
            gz = (int16_t)((gbuf[4] << 8) | gbuf[5]);
        }

        Logger_Log(LOG_INFO,
                   "MPU_GYROTEST_CLK,"
                   "VAL:0x%02X,PWR1_READ:0x%02X,"
                   "GX:%d,GY:%d,GZ:%d,OK:%u",
                   val, pwr1_read, (int)gx, (int)gy, (int)gz,
                   (gst == HAL_OK) ? 1U : 0U);
    }

    /* Restore clock to PLL X gyro (0x01) for remaining tests */
    IMU_MPU9250_FindAndWriteReg(hi2c, MPU9250_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(50);

    /* ── Step 4: Raw 14-byte burst dump from 0x3B ────────────────────── */
    {
        uint8_t buf14[14];
        HAL_StatusTypeDef st = IMU_MPU9250_FindAndReadBytesVerbose(hi2c,
                                                            MPU9250_REG_ACCEL_XOUT_H,
                                                            buf14, 14);
        Logger_Log(LOG_INFO,
                   "MPU_GYROTEST_BURST14,"
                   "HEX:%02X %02X %02X %02X %02X %02X %02X "
                   "%02X %02X %02X %02X %02X %02X %02X,OK:%u",
                   buf14[0],  buf14[1],  buf14[2],  buf14[3],
                   buf14[4],  buf14[5],  buf14[6],  buf14[7],
                   buf14[8],  buf14[9],  buf14[10], buf14[11],
                   buf14[12], buf14[13],
                   (st == HAL_OK) ? 1U : 0U);
    }

    /* ── Step 5: Gyro-only burst read from 0x43 (6 bytes) ────────────── */
    {
        uint8_t gbuf[6];
        HAL_StatusTypeDef st = IMU_MPU9250_FindAndReadBytesVerbose(hi2c,
                                                            MPU9250_REG_GYRO_XOUT_H,
                                                            gbuf, 6);
        int16_t gx = 0, gy = 0, gz = 0;
        if (st == HAL_OK)
        {
            gx = (int16_t)((gbuf[0] << 8) | gbuf[1]);
            gy = (int16_t)((gbuf[2] << 8) | gbuf[3]);
            gz = (int16_t)((gbuf[4] << 8) | gbuf[5]);
        }
        Logger_Log(LOG_INFO,
                   "MPU_GYROTEST_GBURST,"
                   "HEX:%02X %02X %02X %02X %02X %02X,"
                   "GX:%d,GY:%d,GZ:%d,OK:%u",
                   gbuf[0], gbuf[1], gbuf[2],
                   gbuf[3], gbuf[4], gbuf[5],
                   (int)gx, (int)gy, (int)gz,
                   (st == HAL_OK) ? 1U : 0U);
    }

    /* ── Step 6: Individual gyro byte register reads 0x43..0x48 ──────── */
    {
        uint8_t gxh = 0, gxl = 0, gyh = 0, gyl = 0, gzh = 0, gzl = 0;
        IMU_MPU9250_FindAndReadReg(hi2c, 0x43, &gxh);
        IMU_MPU9250_FindAndReadReg(hi2c, 0x44, &gxl);
        IMU_MPU9250_FindAndReadReg(hi2c, 0x45, &gyh);
        IMU_MPU9250_FindAndReadReg(hi2c, 0x46, &gyl);
        IMU_MPU9250_FindAndReadReg(hi2c, 0x47, &gzh);
        IMU_MPU9250_FindAndReadReg(hi2c, 0x48, &gzl);
        Logger_Log(LOG_INFO,
                   "MPU_GYROTEST_GREGS,"
                   "GXH:0x%02X,GXL:0x%02X,"
                   "GYH:0x%02X,GYL:0x%02X,"
                   "GZH:0x%02X,GZL:0x%02X",
                   gxh, gxl, gyh, gyl, gzh, gzl);
    }

    /* ── Step 7: Repeated gyro sampling (20 samples, 20 ms apart) ────── */
    {
        uint8_t any_nonzero = 0;
        int16_t prev_gx = 0, prev_gy = 0, prev_gz = 0;
        uint8_t any_changed = 0;
        uint8_t first = 1;

        for (int n = 0; n < 20; n++)
        {
            uint8_t gbuf[6];
            HAL_StatusTypeDef st = IMU_MPU9250_FindAndReadBytesVerbose(hi2c,
                                                                MPU9250_REG_GYRO_XOUT_H,
                                                                gbuf, 6);
            int16_t gx = 0, gy = 0, gz = 0;
            if (st == HAL_OK)
            {
                gx = (int16_t)((gbuf[0] << 8) | gbuf[1]);
                gy = (int16_t)((gbuf[2] << 8) | gbuf[3]);
                gz = (int16_t)((gbuf[4] << 8) | gbuf[5]);
            }

            Logger_Log(LOG_INFO,
                       "MPU_GYROTEST_SAMPLE,N:%d,GX:%d,GY:%d,GZ:%d",
                       n, (int)gx, (int)gy, (int)gz);

            if (gx != 0 || gy != 0 || gz != 0)
                any_nonzero = 1;

            if (!first)
            {
                if (gx != prev_gx || gy != prev_gy || gz != prev_gz)
                    any_changed = 1;
            }
            prev_gx = gx;
            prev_gy = gy;
            prev_gz = gz;
            first = 0;

            HAL_Delay(20);
        }

        Logger_Log(LOG_INFO,
                   "MPU_GYROTEST_SUMMARY,"
                   "ANY_NONZERO:%u,ANY_CHANGED:%u",
                   any_nonzero, any_changed);
    }

    /* ── Step 8: GYRO_CONFIG full-scale test ─────────────────────────── */
    {
        static const uint8_t gyro_cfg_vals[] = { 0x00, 0x08, 0x10, 0x18 };
        for (size_t i = 0; i < sizeof(gyro_cfg_vals); i++)
        {
            uint8_t val = gyro_cfg_vals[i];
            IMU_MPU9250_FindAndWriteReg(hi2c, MPU9250_REG_GYRO_CONFIG, val);
            HAL_Delay(10);

            uint8_t readback = 0;
            IMU_MPU9250_FindAndReadReg(hi2c, MPU9250_REG_GYRO_CONFIG, &readback);

            uint8_t gbuf[6];
            HAL_StatusTypeDef gst = IMU_MPU9250_FindAndReadBytesVerbose(hi2c,
                                                                 MPU9250_REG_GYRO_XOUT_H,
                                                                 gbuf, 6);
            int16_t gx = 0, gy = 0, gz = 0;
            if (gst == HAL_OK)
            {
                gx = (int16_t)((gbuf[0] << 8) | gbuf[1]);
                gy = (int16_t)((gbuf[2] << 8) | gbuf[3]);
                gz = (int16_t)((gbuf[4] << 8) | gbuf[5]);
            }

            Logger_Log(LOG_INFO,
                       "MPU_GYROTEST_CFG,"
                       "VAL:0x%02X,READ:0x%02X,"
                       "GX:%d,GY:%d,GZ:%d,OK:%u",
                       val, readback, (int)gx, (int)gy, (int)gz,
                       (gst == HAL_OK) ? 1U : 0U);
        }
    }

    /* Restore GYRO_CONFIG to default ±250 dps */
    IMU_MPU9250_FindAndWriteReg(hi2c, MPU9250_REG_GYRO_CONFIG, 0x00);

    Logger_Log(LOG_INFO, "MPU_GYROTEST,DONE");
}

/* ── Stage 6: read and convert to physical units (scaled integer) ────────── */

/* Conversion constants for current config:
 *   ACCEL_CONFIG = 0x00 -> ±2g  -> 16384 LSB/g
 *   GYRO_CONFIG  = 0x00 -> ±250 dps -> 131 LSB/dps
 *   Temperature: temp_c = raw/333.87 + 21.0  (MPU6500/9250 family) */
#define IMU_ACCEL_LSB_PER_G    16384
#define IMU_GYRO_LSB_PER_DPS   131
#define IMU_TEMP_DIV_X10000    33387   /* 333.87 * 100 */

HAL_StatusTypeDef IMU_MPU9250_ReadConverted(I2C_HandleTypeDef *hi2c,
                                            IMU_MPU9250_Conv_t *conv)
{
    IMU_MPU9250_Raw_t raw;
    HAL_StatusTypeDef st = IMU_MPU9250_ReadRaw(hi2c, &raw);
    if (st != HAL_OK)
        return HAL_ERROR;

    conv->acc_x_mg    = ((int32_t)raw.acc_x * 1000) / IMU_ACCEL_LSB_PER_G;
    conv->acc_y_mg    = ((int32_t)raw.acc_y * 1000) / IMU_ACCEL_LSB_PER_G;
    conv->acc_z_mg    = ((int32_t)raw.acc_z * 1000) / IMU_ACCEL_LSB_PER_G;

    conv->temp_cx100  = (((int32_t)raw.temp * 10000) / IMU_TEMP_DIV_X10000) + 2100;

    conv->gyro_x_mdps = ((int32_t)raw.gyro_x * 1000) / IMU_GYRO_LSB_PER_DPS;
    conv->gyro_y_mdps = ((int32_t)raw.gyro_y * 1000) / IMU_GYRO_LSB_PER_DPS;
    conv->gyro_z_mdps = ((int32_t)raw.gyro_z * 1000) / IMU_GYRO_LSB_PER_DPS;

    return HAL_OK;
}
