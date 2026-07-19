/* QMC5883L magnetometer driver with fault-tolerant state machine.
 * Filename retained as mag_qmc5883p.c; all symbols corrected for QMC5883L. */

#include "mag_qmc5883p.h"
#include "i2c_recovery.h"
#include "logger.h"
#include <string.h>
#include <stdlib.h>

/* ── Integer square root (Babylonian, 64-bit) ─────────────────────────────── */
static uint32_t isqrt_u64(uint64_t n)
{
    if (n == 0) return 0;
    uint64_t x = n;
    uint64_t y = (x + 1) / 2;
    while (y < x) { x = y; y = (x + n / x) / 2; }
    return (uint32_t)x;
}

/* ── Runtime reset helper ─────────────────────────────────────────────────── */

void MAG_QMC5883L_RuntimeReset(uint8_t preserve_diagnostics)
{
    MAG_QMC5883L_Handle_t *h = &g_mag_handle;

    /* Always reset transient state */
    h->state = MAG_STATE_OFFLINE;
    h->found = 0;
    h->initialized = 0;
    h->has_valid_data = 0;
    h->valid_sample_streak = 0;
    h->consecutive_comm_errors = 0;
    h->consecutive_drdy_timeouts = 0;
    h->consecutive_invalid_samples = 0;
    h->probe_failure_count = 0;
    h->verify_attempts = 0;
    h->verify_mismatch_count = 0;
    h->recovery_pending = 0;
    h->init_step = 0;

    /* Timestamps */
    uint32_t now = HAL_GetTick();
    h->state_tick = now;
    h->last_reconnect_tick = 0;         /* allow immediate probe */
    h->last_acquire_tick = 0;
    h->last_telemetry_tick = 0;
    h->deadline_tick = 0;
    h->init_deadline_tick = 0;

    /* Cached sample invalidation */
    h->last_valid_x = 0;
    h->last_valid_y = 0;
    h->last_valid_z = 0;
    h->last_valid_status = 0;
    h->last_valid_tick = 0;
    h->last_fresh_sample_tick = 0;
    h->identical_sample_count = 0;

    /* Register readback cache */
    h->ctrl1_readback = 0;
    h->setreset_readback = 0;

    /* HAL status */
    h->last_hal_status = HAL_OK;
    h->last_hal_error = 0;

    /* DRDY excessive timeout latch */
    h->drdy_excessive_latched = 0;

    /* Lifetime diagnostics: optionally preserve */
    if (!preserve_diagnostics)
    {
        h->total_comm_errors = 0;
        h->total_drdy_timeouts = 0;
        h->total_invalid_samples = 0;
        h->total_overflows = 0;
        h->total_dor_events = 0;
        h->reconnect_count = 0;
        h->recovery_count = 0;
    }
}

/* ── Global handle ───────────────────────────────────────────────────────── */
MAG_QMC5883L_Handle_t g_mag_handle = {0};

/* ── State names for logging ─────────────────────────────────────────────── */
static const char *MagStateName(MagState_t s)
{
    switch (s)
    {
        case MAG_STATE_OFFLINE:        return "OFFLINE";
        case MAG_STATE_RECOVERING:     return "RECOVERING";
        case MAG_STATE_PROBING:        return "PROBING";
        case MAG_STATE_RESETTING:      return "RESETTING";
        case MAG_STATE_WAIT_RESET:     return "WAIT_RESET";
        case MAG_STATE_SETRESET:       return "SETRESET";
        case MAG_STATE_WAIT_SETRESET:  return "WAIT_SETRESET";
        case MAG_STATE_CTRL1:          return "CTRL1";
        case MAG_STATE_WAIT_CTRL1:     return "WAIT_CTRL1";
        case MAG_STATE_VERIFY:         return "VERIFY";
        case MAG_STATE_STABILIZING:    return "STABILIZING";
        case MAG_STATE_ONLINE:         return "ONLINE";
        case MAG_STATE_FAULT:          return "FAULT";
        default:                       return "UNKNOWN";
    }
}

/* ── State transition helper ─────────────────────────────────────────────── */
static void Mag_SetState(MAG_QMC5883L_Handle_t *h, MagState_t new_state)
{
    if (h->state != new_state)
    {
        Logger_Log(LOG_INFO, "MAG_STATE,%s", MagStateName(new_state));
    }
    h->state = new_state;
    h->state_tick = HAL_GetTick();
}

/* ── Low-level I2C helpers ───────────────────────────────────────────────── */

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

/* ── Public register read (legacy) ───────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    return Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, reg, value);
}

/* ── Tick-wrap-safe deadline check ───────────────────────────────────────── */
static inline bool TickDeadlineReached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

/* ── Invalid / overflow sample detection ────────────────────────────────── */

static uint8_t Mag_IsInvalidSample(int16_t x, int16_t y, int16_t z, uint8_t status)
{
    /* Overflow flag */
    if (status & MAG_QMC5883L_STATUS_OVFL)
        return 1;

    /* All zero */
    if (x == 0 && y == 0 && z == 0)
        return 1;

    /* All -1 (0xFFFF signed) — common I2C error pattern */
    if (x == -1 && y == -1 && z == -1)
        return 1;

    /* Any axis at INT16_MIN or INT16_MAX (saturation / error) */
    if (x == INT16_MAX || x == INT16_MIN ||
        y == INT16_MAX || y == INT16_MIN ||
        z == INT16_MAX || z == INT16_MIN)
        return 1;

    /* Absolute sum too small (degenerate field) */
    int32_t abs_sum = (int32_t)abs(x) + (int32_t)abs(y) + (int32_t)abs(z);
    if (abs_sum < 3)
        return 1;

    return 0;
}

/* ── Error classification helpers ────────────────────────────────────────── */

static void Mag_RecordCommError(MAG_QMC5883L_Handle_t *h,
                                HAL_StatusTypeDef hal_st, uint32_t hal_err)
{
    h->last_hal_status = hal_st;
    h->last_hal_error  = hal_err;
    h->total_comm_errors++;
    h->consecutive_comm_errors++;
    h->consecutive_drdy_timeouts = 0;  /* reset DRDY counter on comm error */
    h->valid_sample_streak = 0;

    Logger_Log(LOG_INFO,
               "MAG_COMM_ERROR,COUNT:%lu,CONSEC:%u,HAL:%d,ERR:%lu",
               (unsigned long)h->total_comm_errors,
               (unsigned)h->consecutive_comm_errors,
               (int)hal_st, (unsigned long)hal_err);
}

static void Mag_RecordDrdyTimeout(MAG_QMC5883L_Handle_t *h)
{
    h->total_drdy_timeouts++;
    h->consecutive_drdy_timeouts++;
    /* Do NOT reset consecutive_comm_errors here.
     * DRDY=0 is not a communication failure — it means the sensor hasn't
     * produced a new sample yet.  Clearing comm errors here would mask
     * intermittent I2C problems: I2C error -> comm=1, status read OK
     * DRDY=0 -> comm=0, repeating forever. */

    Logger_Log(LOG_INFO,
               "MAG_DRDY_TIMEOUT,COUNT:%lu,CONSEC:%u",
               (unsigned long)h->total_drdy_timeouts,
               (unsigned)h->consecutive_drdy_timeouts);
}

static void Mag_RecordDorEvent(MAG_QMC5883L_Handle_t *h)
{
    h->total_dor_events++;
    /* DOR (data overrun) means the sensor produced a new sample before the
     * previous one was read.  This is a diagnostic indicator only — it is
     * NOT a communication failure and does NOT invalidate the data.
     * Expected at 200 Hz sensor ODR with 50 Hz firmware read rate. */
}

static void Mag_RecordInvalidSample(MAG_QMC5883L_Handle_t *h,
                                    int16_t x, int16_t y, int16_t z, uint8_t status)
{
    h->total_invalid_samples++;
    h->consecutive_invalid_samples++;
    h->valid_sample_streak = 0;

    if (status & MAG_QMC5883L_STATUS_OVFL)
    {
        h->total_overflows++;
        Logger_Log(LOG_INFO, "MAG_OVERFLOW,COUNT:%lu", (unsigned long)h->total_overflows);
    }

    Logger_Log(LOG_INFO,
               "MAG_INVALID_SAMPLE,X:%d,Y:%d,Z:%d,STATUS:0x%02X,COUNT:%lu",
               (int)x, (int)y, (int)z, (unsigned)status,
               (unsigned long)h->total_invalid_samples);
}

static void Mag_RecordValidSample(MAG_QMC5883L_Handle_t *h)
{
    /* Called ONLY when a truly valid DRDY-backed sample is stored.
     * This is the only event that clears the consecutive comm error counter,
     * preventing the ping-pong pattern where intermittent I2C errors are
     * masked by a single successful status read with DRDY=0. */
    h->consecutive_comm_errors = 0;
    h->last_success_tick = HAL_GetTick();
}

/* ── Detection (probe + chip ID read) ────────────────────────────────────── */

static HAL_StatusTypeDef Mag_DetectInternal(I2C_HandleTypeDef *hi2c,
                                            MAG_QMC5883L_Handle_t *h)
{
    h->found = 0;
    h->chip_id = 0;
    h->last_hal_status = HAL_OK;
    h->last_hal_error  = 0;

    /* Probe address */
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(hi2c, MAG_QMC5883L_DEVADDR_HAL,
                                                  2, MAG_I2C_TIMEOUT_MS);
    if (st != HAL_OK)
    {
        h->last_hal_status = st;
        h->last_hal_error  = HAL_I2C_GetError(hi2c);
        Logger_Log(LOG_INFO, "MAG_PROBE,HAL:%d,ERR:%lu",
                   (int)st, (unsigned long)h->last_hal_error);
        return st;
    }

    /* Read chip ID */
    uint8_t chip_id = 0;
    st = Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_CHIP_ID, &chip_id);
    if (st != HAL_OK)
    {
        h->last_hal_status = st;
        h->last_hal_error  = HAL_I2C_GetError(hi2c);
        Logger_Log(LOG_INFO, "MAG_CHIPID_READ_FAIL,HAL:%d,ERR:%lu",
                   (int)st, (unsigned long)h->last_hal_error);
        return st;
    }

    h->chip_id = chip_id;

#if MAG_STRICT_CHIP_ID_CHECK
    if (chip_id != MAG_QMC5883L_CHIP_ID_EXPECTED)
    {
        Logger_Log(LOG_INFO, "MAG_CHIPID,EXPECTED:0x%02X,GOT:0x%02X,OK:0",
                   (unsigned)MAG_QMC5883L_CHIP_ID_EXPECTED, (unsigned)chip_id);
        h->found = 0;
        return HAL_ERROR;
    }
#else
    Logger_Log(LOG_INFO, "MAG_CHIPID,GOT:0x%02X,STRICT:0", (unsigned)chip_id);
#endif

    h->found = 1;
    h->addr7 = MAG_QMC5883L_ADDR7;
    Logger_Log(LOG_INFO, "MAG_PROBE,ADDR:0x%02X,CHIP_ID:0x%02X,OK:1",
               (unsigned)h->addr7, (unsigned)chip_id);
    return HAL_OK;
}

/* ── Non-blocking state machine ──────────────────────────────────────────── */

void MAG_QMC5883L_Task(I2C_HandleTypeDef *hi2c)
{
    MAG_QMC5883L_Handle_t *h = &g_mag_handle;
    uint32_t now = HAL_GetTick();

    switch (h->state)
    {
    case MAG_STATE_OFFLINE:
    {
        /* Use longer interval after failed recovery, normal interval otherwise */
        uint32_t interval = h->recovery_pending ? MAG_RECOVERY_RETRY_INTERVAL_MS
                                                : MAG_RECONNECT_INTERVAL_MS;
        if ((now - h->last_reconnect_tick) < interval)
            return;
        h->last_reconnect_tick = now;
        h->recovery_pending = 0;
        h->reconnect_count++;
        h->found = 0;
        h->initialized = 0;
        Logger_Log(LOG_INFO, "MAG_RECONNECT,ATTEMPT:%lu", (unsigned long)h->reconnect_count);
        Mag_SetState(h, MAG_STATE_PROBING);
        __attribute__((fallthrough));
    }

    case MAG_STATE_PROBING:
    {
        HAL_StatusTypeDef st = Mag_DetectInternal(hi2c, h);
        if (st != HAL_OK)
        {
            h->found = 0;
            h->initialized = 0;
            h->probe_failure_count++;
            Logger_Log(LOG_INFO, "MAG_PROBE_FAIL,COUNT:%u,THRESH:%u",
                       (unsigned)h->probe_failure_count,
                       (unsigned)MAG_MAX_PROBE_FAILURES);

            if (h->probe_failure_count >= MAG_MAX_PROBE_FAILURES)
            {
                /* Classify the probe failure to decide recovery policy.
                 *
                 * NACK/AF with lines high and peripheral READY:
                 *   Device is simply absent — not a bus fault.
                 *   Stay OFFLINE, normal reconnect cycle, NO bus recovery.
                 *   This prevents resetting the MPU9250 on the shared bus
                 *   every time the magnetometer is disconnected.
                 *
                 * HAL_BUSY, HAL_TIMEOUT, BERR, ARLO, or lines low:
                 *   Bus or peripheral is stuck — full recovery needed. */

                GPIO_PinState sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
                GPIO_PinState scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
                HAL_I2C_StateTypeDef i2c_state = HAL_I2C_GetState(hi2c);
                uint32_t hal_err = h->last_hal_error;
                HAL_StatusTypeDef hal_st = h->last_hal_status;

                /* NACK (AF) is the expected error when device is absent.
                 * With lines high and peripheral READY, no recovery needed. */
                uint8_t nack_only = (hal_st == HAL_ERROR &&
                                     (hal_err & HAL_I2C_ERROR_AF)) ? 1U : 0U;
                uint8_t lines_high = (sda == GPIO_PIN_SET && scl == GPIO_PIN_SET) ? 1U : 0U;
                uint8_t periph_ready = (i2c_state == HAL_I2C_STATE_READY) ? 1U : 0U;

                uint8_t device_absent = (nack_only && lines_high && periph_ready);

                /* Bus/peripheral fault indicators */
                uint8_t bus_stuck = (!lines_high) ? 1U : 0U;
                uint8_t periph_stuck = (i2c_state == HAL_I2C_STATE_BUSY ||
                                        i2c_state == HAL_I2C_STATE_BUSY_TX ||
                                        i2c_state == HAL_I2C_STATE_BUSY_RX) ? 1U : 0U;
                uint8_t bus_error = (hal_err & (HAL_I2C_ERROR_BERR |
                                                HAL_I2C_ERROR_ARLO |
                                                HAL_I2C_ERROR_OVR)) ? 1U : 0U;
                uint8_t timeout_err = (hal_st == HAL_TIMEOUT) ? 1U : 0U;

                Logger_Log(LOG_INFO,
                           "MAG_PROBE_CLASSIFY,NACK:%u,LINES_H:%u,PERIPH_RDY:%u,"
                           "BUS_STUCK:%u,PERIPH_ST:%u,BUS_ERR:%u,TIMEOUT:%u",
                           nack_only, lines_high, periph_ready,
                           bus_stuck, periph_stuck, bus_error, timeout_err);

                h->probe_failure_count = 0;

                if (device_absent && !bus_stuck && !periph_stuck && !bus_error && !timeout_err)
                {
                    /* Device absent, bus healthy — normal reconnect cycle.
                     * Do NOT trigger bus recovery. The MPU9250 on the shared
                     * bus continues to operate normally. */
                    Logger_Log(LOG_INFO, "MAG_PROBE_ABSENT,NO_RECOVERY");
                    Mag_SetState(h, MAG_STATE_OFFLINE);
                }
                else
                {
                    /* Bus or peripheral fault — full recovery required */
                    Logger_Log(LOG_INFO, "MAG_PROBE_EXCESSIVE,BUS_RECOVERY");
                    Mag_SetState(h, MAG_STATE_RECOVERING);
                }
            }
            else
            {
                Mag_SetState(h, MAG_STATE_OFFLINE);
            }
            return;
        }
        h->probe_failure_count = 0;
        h->init_deadline_tick = now + MAG_INIT_TIMEOUT_MS;
        Mag_SetState(h, MAG_STATE_RESETTING);
        return;
    }

    /* ── Non-blocking init: each command + wait is a separate state ── */

    case MAG_STATE_RESETTING:
    {
        /* Init deadline check — if we've been in init too long, abort */
        if (h->init_deadline_tick && TickDeadlineReached(now, h->init_deadline_tick))
        {
            Logger_Log(LOG_INFO, "MAG_INIT_TIMEOUT,STATE:RESETTING");
            h->found = 0; h->initialized = 0;
            Mag_SetState(h, MAG_STATE_RECOVERING);
            return;
        }
        HAL_StatusTypeDef st = Mag_WriteReg(hi2c, MAG_QMC5883L_ADDR7,
                                            MAG_QMC5883L_REG_CTRL2, 0x80);
        if (st != HAL_OK)
        {
            Mag_RecordCommError(h, st, HAL_I2C_GetError(hi2c));
            if (h->consecutive_comm_errors >= MAG_MAX_CONSECUTIVE_COMM_ERRORS)
            {
                h->found = 0; h->initialized = 0;
                Mag_SetState(h, MAG_STATE_RECOVERING);
            }
            return;
        }
        Logger_Log(LOG_INFO, "MAG_RESET,OK:1");
        h->deadline_tick = now + 20;
        Mag_SetState(h, MAG_STATE_WAIT_RESET);
        return;
    }

    case MAG_STATE_WAIT_RESET:
        if (!TickDeadlineReached(now, h->deadline_tick))
            return;
        Mag_SetState(h, MAG_STATE_SETRESET);
        return;

    case MAG_STATE_SETRESET:
    {
        if (h->init_deadline_tick && TickDeadlineReached(now, h->init_deadline_tick))
        {
            Logger_Log(LOG_INFO, "MAG_INIT_TIMEOUT,STATE:SETRESET");
            h->found = 0; h->initialized = 0;
            Mag_SetState(h, MAG_STATE_RECOVERING);
            return;
        }
        HAL_StatusTypeDef st = Mag_WriteReg(hi2c, MAG_QMC5883L_ADDR7,
                                            MAG_QMC5883L_REG_SET_RESET, 0x01);
        if (st != HAL_OK)
        {
            Mag_RecordCommError(h, st, HAL_I2C_GetError(hi2c));
            if (h->consecutive_comm_errors >= MAG_MAX_CONSECUTIVE_COMM_ERRORS)
            {
                h->found = 0; h->initialized = 0;
                Mag_SetState(h, MAG_STATE_RECOVERING);
            }
            return;
        }
        h->deadline_tick = now + 10;
        Mag_SetState(h, MAG_STATE_WAIT_SETRESET);
        return;
    }

    case MAG_STATE_WAIT_SETRESET:
        if (!TickDeadlineReached(now, h->deadline_tick))
            return;
        Mag_SetState(h, MAG_STATE_CTRL1);
        return;

    case MAG_STATE_CTRL1:
    {
        if (h->init_deadline_tick && TickDeadlineReached(now, h->init_deadline_tick))
        {
            Logger_Log(LOG_INFO, "MAG_INIT_TIMEOUT,STATE:CTRL1");
            h->found = 0; h->initialized = 0;
            Mag_SetState(h, MAG_STATE_RECOVERING);
            return;
        }
        HAL_StatusTypeDef st = Mag_WriteReg(hi2c, MAG_QMC5883L_ADDR7,
                                            MAG_QMC5883L_REG_CTRL1,
                                            MAG_QMC5883L_CTRL1_VALUE);
        if (st != HAL_OK)
        {
            Mag_RecordCommError(h, st, HAL_I2C_GetError(hi2c));
            if (h->consecutive_comm_errors >= MAG_MAX_CONSECUTIVE_COMM_ERRORS)
            {
                h->found = 0; h->initialized = 0;
                Mag_SetState(h, MAG_STATE_RECOVERING);
            }
            return;
        }
        h->deadline_tick = now + 20;
        Mag_SetState(h, MAG_STATE_WAIT_CTRL1);
        return;
    }

    case MAG_STATE_WAIT_CTRL1:
        if (!TickDeadlineReached(now, h->deadline_tick))
            return;
        Mag_SetState(h, MAG_STATE_VERIFY);
        return;

    case MAG_STATE_VERIFY:
    {
        /* Init deadline check */
        if (h->init_deadline_tick && TickDeadlineReached(now, h->init_deadline_tick))
        {
            Logger_Log(LOG_INFO, "MAG_INIT_TIMEOUT,STATE:VERIFY");
            h->found = 0; h->initialized = 0;
            Mag_SetState(h, MAG_STATE_RECOVERING);
            return;
        }

        uint8_t ctrl1_rb = 0, setreset_rb = 0;
        HAL_StatusTypeDef st;
        st = Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_CTRL1, &ctrl1_rb);
        if (st != HAL_OK)
        {
            Mag_RecordCommError(h, st, HAL_I2C_GetError(hi2c));
            h->verify_attempts++;
            Logger_Log(LOG_INFO, "MAG_VERIFY_FAIL,REASON:CTRL1_READ,ATTEMPT:%u/%u",
                       (unsigned)h->verify_attempts, (unsigned)MAG_MAX_VERIFY_ATTEMPTS);
            if (h->verify_attempts >= MAG_MAX_VERIFY_ATTEMPTS)
            {
                Logger_Log(LOG_INFO, "MAG_VERIFY_EXCEEDED,RECOVERY");
                h->verify_attempts = 0;
                h->found = 0; h->initialized = 0;
                Mag_SetState(h, MAG_STATE_RECOVERING);
            }
            return;
        }
        st = Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_SET_RESET, &setreset_rb);
        if (st != HAL_OK)
        {
            Mag_RecordCommError(h, st, HAL_I2C_GetError(hi2c));
            h->verify_attempts++;
            Logger_Log(LOG_INFO, "MAG_VERIFY_FAIL,REASON:SETRESET_READ,ATTEMPT:%u/%u",
                       (unsigned)h->verify_attempts, (unsigned)MAG_MAX_VERIFY_ATTEMPTS);
            if (h->verify_attempts >= MAG_MAX_VERIFY_ATTEMPTS)
            {
                Logger_Log(LOG_INFO, "MAG_VERIFY_EXCEEDED,RECOVERY");
                h->verify_attempts = 0;
                h->found = 0; h->initialized = 0;
                Mag_SetState(h, MAG_STATE_RECOVERING);
            }
            return;
        }

        h->ctrl1_readback    = ctrl1_rb;
        h->setreset_readback = setreset_rb;

        uint8_t ctrl1_ok    = (ctrl1_rb == MAG_QMC5883L_CTRL1_VALUE) ? 1U : 0U;
        uint8_t setreset_ok = (setreset_rb == 0x01) ? 1U : 0U;

        Logger_Log(LOG_INFO,
                   "MAG_CONFIG,CTRL1:0x%02X,SETRESET:0x%02X,READBACK_OK:%u",
                   (unsigned)ctrl1_rb, (unsigned)setreset_rb,
                   (ctrl1_ok && setreset_ok) ? 1U : 0U);

        if (!ctrl1_ok || !setreset_ok)
        {
            /* Register read succeeded but value mismatch — NOT a communication
             * error.  This is a configuration/verification failure.
             * Track separately from comm errors for accurate diagnostics. */
            h->verify_attempts++;
            h->verify_mismatch_count++;
            Logger_Log(LOG_INFO,
                       "MAG_VERIFY_FAIL,REASON:MISMATCH,ATTEMPT:%u/%u,"
                       "CTRL1:0x%02X,SR:0x%02X,MISMATCH_TOTAL:%u",
                       (unsigned)h->verify_attempts, (unsigned)MAG_MAX_VERIFY_ATTEMPTS,
                       (unsigned)ctrl1_rb, (unsigned)setreset_rb,
                       (unsigned)h->verify_mismatch_count);
            if (h->verify_attempts >= MAG_MAX_VERIFY_ATTEMPTS)
            {
                Logger_Log(LOG_INFO, "MAG_VERIFY_EXCEEDED,RECOVERY,MISMATCH_TOTAL:%u",
                           (unsigned)h->verify_mismatch_count);
                h->verify_attempts = 0;
                h->found = 0; h->initialized = 0;
                Mag_SetState(h, MAG_STATE_RECOVERING);
            }
            return;
        }

        /* Verification passed */
        h->verify_attempts = 0;
        h->initialized = 1;
        h->valid_sample_streak = 0;
        h->consecutive_invalid_samples = 0;
        h->identical_sample_count = 0;
        h->last_fresh_sample_tick = now;
        Mag_SetState(h, MAG_STATE_STABILIZING);
        return;
    }

    /* ── Stabilizing: wait for valid samples ──────────────────────── */

    case MAG_STATE_STABILIZING:
    {
        /* Rate-limit acquisition */
        if ((now - h->last_acquire_tick) >= MAG_ACQUIRE_INTERVAL_MS)
        {
            h->last_acquire_tick = now;
            MAG_QMC5883L_Acquire(hi2c);
        }

        if (h->valid_sample_streak >= MAG_REQUIRED_VALID_SAMPLES)
        {
            Mag_SetState(h, MAG_STATE_ONLINE);
            Logger_Log(LOG_INFO, "MAG_ONLINE");
            return;
        }
        if ((now - h->state_tick) > 5000)
        {
            Logger_Log(LOG_INFO, "MAG_STABILIZE_TIMEOUT,VALID_STREAK:%u,CONSEC_COMM:%u",
                       (unsigned)h->valid_sample_streak,
                       (unsigned)h->consecutive_comm_errors);
            if (h->consecutive_comm_errors >= MAG_MAX_CONSECUTIVE_COMM_ERRORS)
            {
                /* Communication failures — recover bus */
                h->found = 0; h->initialized = 0;
                Mag_SetState(h, MAG_STATE_RECOVERING);
            }
            else
            {
                /* Bus OK but no data — soft reset sensor */
                h->initialized = 0;
                h->valid_sample_streak = 0;
                Mag_SetState(h, MAG_STATE_RESETTING);
            }
        }
        return;
    }

    /* ── Online: rate-limited acquisition ─────────────────────────── */

    case MAG_STATE_ONLINE:
    {
        if ((now - h->last_acquire_tick) < MAG_ACQUIRE_INTERVAL_MS)
            return;
        h->last_acquire_tick = now;

        MAG_QMC5883L_Acquire(hi2c);

        /* Refresh now after Acquire's I2C transactions.  Acquire() sets
         * last_fresh_sample_tick to HAL_GetTick() which may be several ms
         * after the now captured at the top of Task().  Without this refresh,
         * the age check below would subtract a newer tick from an older now,
         * producing unsigned underflow (~0xFFFFFFFE) and a spurious timeout. */
        now = HAL_GetTick();

        /* Repeated HAL communication failures → bus recovery */
        if (h->consecutive_comm_errors >= MAG_MAX_CONSECUTIVE_COMM_ERRORS)
        {
            Logger_Log(LOG_INFO, "MAG_COMM_EXCESSIVE,CONSEC:%u,RECOVERING",
                       (unsigned)h->consecutive_comm_errors);
            h->found = 0; h->initialized = 0; h->valid_sample_streak = 0;
            Mag_SetState(h, MAG_STATE_RECOVERING);
            return;
        }

        /* Data timeout: no fresh DRDY-backed sample for too long.
         * Only reached when comm is healthy → sensor soft reset. */
        if (h->has_valid_data &&
            (uint32_t)(now - h->last_fresh_sample_tick) >= MAG_DATA_TIMEOUT_MS)
        {
            Logger_Log(LOG_INFO, "MAG_DATA_TIMEOUT,AGE_MS:%lu,SOFT_RESET",
                       (unsigned long)(now - h->last_fresh_sample_tick));
            h->initialized = 0;
            h->valid_sample_streak = 0;
            Mag_SetState(h, MAG_STATE_RESETTING);
            return;
        }

        return;
    }

    case MAG_STATE_RECOVERING:
    {
        Logger_Log(LOG_INFO, "I2C_RECOVERY,REASON:MAG_COMM");
        h->recovery_count++;
        HAL_StatusTypeDef rec_st = I2C_BusRecovery(hi2c);

        /* Always clear transient state after recovery attempt */
        h->consecutive_comm_errors = 0;
        h->consecutive_drdy_timeouts = 0;
        h->consecutive_invalid_samples = 0;
        h->valid_sample_streak = 0;
        h->probe_failure_count = 0;
        h->verify_attempts = 0;
        h->verify_mismatch_count = 0;
        h->found = 0;
        h->initialized = 0;

        if (rec_st == HAL_OK)
        {
            Logger_Log(LOG_INFO, "MAG_RECOVERY,OK:1,COUNT:%lu",
                       (unsigned long)h->recovery_count);
            /* Recovery succeeded — probe immediately */
            h->last_reconnect_tick = 0;
            h->recovery_pending = 0;
            Mag_SetState(h, MAG_STATE_OFFLINE);
        }
        else
        {
            Logger_Log(LOG_INFO, "MAG_RECOVERY,OK:0,COUNT:%lu,HAL:%d,WAIT:%lums",
                       (unsigned long)h->recovery_count, (int)rec_st,
                       (unsigned long)MAG_RECOVERY_RETRY_INTERVAL_MS);
            /* Recovery failed — back off before retrying */
            h->last_reconnect_tick = now;
            h->recovery_pending = 1;
            Mag_SetState(h, MAG_STATE_OFFLINE);
        }
        return;
    }

    case MAG_STATE_FAULT:
    {
        if ((now - h->last_reconnect_tick) < (MAG_RECONNECT_INTERVAL_MS * 4))
            return;
        h->last_reconnect_tick = now;
        Mag_SetState(h, MAG_STATE_OFFLINE);
        return;
    }

    default:
        Mag_SetState(h, MAG_STATE_OFFLINE);
        return;
    }
}

/* ── Non-blocking sample acquisition ─────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883L_Acquire(I2C_HandleTypeDef *hi2c)
{
    MAG_QMC5883L_Handle_t *h = &g_mag_handle;

    if (!h->initialized || !h->found)
        return HAL_ERROR;

    /* Read status register once (non-blocking) */
    uint8_t status = 0;
    HAL_StatusTypeDef st = Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7,
                                       MAG_QMC5883L_REG_STATUS, &status);
    if (st != HAL_OK)
    {
        Mag_RecordCommError(h, st, HAL_I2C_GetError(hi2c));
        return HAL_ERROR;
    }

    /* Status read succeeded — but do NOT clear consecutive_comm_errors here.
     * Only a valid DRDY-backed sample clears the comm error counter.
     * This prevents the ping-pong pattern: I2C error -> comm=1, status OK
     * DRDY=0 -> comm=0, I2C error -> comm=1, ... */

    /* Track DOR (data overrun) — diagnostic only, not an error */
    if (status & MAG_QMC5883L_STATUS_DOR)
    {
        Mag_RecordDorEvent(h);
    }

    /* Check DRDY — if not ready, just return (no blocking wait) */
    if (!(status & MAG_QMC5883L_STATUS_DRDY))
    {
        Mag_RecordDrdyTimeout(h);

        /* Check DRDY timeout threshold — may indicate sensor stuck.
         * Log only on the threshold crossing to avoid spam. */
        if (h->consecutive_drdy_timeouts >= MAG_MAX_CONSECUTIVE_DRDY_TIMEOUTS &&
            !h->drdy_excessive_latched)
        {
            h->drdy_excessive_latched = 1;
            Logger_Log(LOG_INFO, "MAG_DRDY_EXCESSIVE,CONSEC:%u,VERIFY_REGISTERS",
                       (unsigned)h->consecutive_drdy_timeouts);

            /* Verify CTRL1 and SETRESET register values */
            uint8_t ctrl1_rb = 0, setreset_rb = 0;
            HAL_StatusTypeDef st_c1 = Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7,
                                                   MAG_QMC5883L_REG_CTRL1, &ctrl1_rb);
            HAL_StatusTypeDef st_sr = Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7,
                                                   MAG_QMC5883L_REG_SET_RESET, &setreset_rb);

            if (st_c1 != HAL_OK || st_sr != HAL_OK)
            {
                /* I2C register read failed — bus/peripheral problem.
                 * Comm error already recorded by the state machine.
                 * The consecutive_comm_errors counter will drive recovery
                 * via the normal comm error escalation path. */
                Logger_Log(LOG_INFO, "MAG_DRDY_VERIFY,COMM_FAIL,C1:%d,SR:%d",
                           (int)st_c1, (int)st_sr);
            }
            else if (ctrl1_rb != MAG_QMC5883L_CTRL1_VALUE ||
                     setreset_rb != 0x01)
            {
                /* Register values lost — sensor configuration corrupted.
                 * Schedule sensor re-init via soft reset. */
                Logger_Log(LOG_INFO,
                           "MAG_DRDY_VERIFY,CONFIG_LOST,"
                           "CTRL1:0x%02X(EXPECT:0x%02X),SR:0x%02X(EXPECT:0x01)",
                           (unsigned)ctrl1_rb, (unsigned)MAG_QMC5883L_CTRL1_VALUE,
                           (unsigned)setreset_rb);
                h->initialized = 0;
                h->valid_sample_streak = 0;
                Mag_SetState(h, MAG_STATE_RESETTING);
                return HAL_BUSY;
            }
            else
            {
                /* Registers correct but sensor not producing data.
                 * Soft reset and re-init to recover sensor state. */
                Logger_Log(LOG_INFO,
                           "MAG_DRDY_VERIFY,REGISTERS_OK,CTRL1:0x%02X,SR:0x%02X,SOFT_RESET",
                           (unsigned)ctrl1_rb, (unsigned)setreset_rb);
                h->initialized = 0;
                h->valid_sample_streak = 0;
                Mag_SetState(h, MAG_STATE_RESETTING);
                return HAL_BUSY;
            }
        }
        return HAL_BUSY;
    }

    /* Read 6 data bytes */
    uint8_t buf[6];
    st = Mag_ReadBytes(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_X_LSB, buf, 6);
    if (st != HAL_OK)
    {
        Mag_RecordCommError(h, st, HAL_I2C_GetError(hi2c));
        return HAL_ERROR;
    }

    /* Decode little-endian signed 16-bit */
    int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

    /* Validate sample */
    if (Mag_IsInvalidSample(x, y, z, status))
    {
        Mag_RecordInvalidSample(h, x, y, z, status);

        /* Check invalid sample threshold */
        if (h->consecutive_invalid_samples >= MAG_MAX_CONSECUTIVE_INVALID)
        {
            Logger_Log(LOG_INFO, "MAG_INVALID_EXCESSIVE,CONSEC:%u,RECOVERY",
                       (unsigned)h->consecutive_invalid_samples);
            h->found = 0; h->initialized = 0;
            Mag_SetState(h, MAG_STATE_RECOVERING);
        }
        return HAL_ERROR;
    }

    /* Valid DRDY-backed sample — always accept, even if values unchanged.
     * A stationary sensor legitimately produces identical samples. */
    if (h->has_valid_data &&
        x == h->last_valid_x && y == h->last_valid_y && z == h->last_valid_z)
    {
        h->identical_sample_count++;
    }

    h->last_valid_x      = x;
    h->last_valid_y      = y;
    h->last_valid_z      = z;
    h->last_valid_status = status;
    h->last_valid_tick   = HAL_GetTick();
    h->last_fresh_sample_tick = h->last_valid_tick;
    h->has_valid_data    = 1;
    h->valid_sample_streak++;
    h->consecutive_invalid_samples = 0;
    h->consecutive_drdy_timeouts = 0;
    h->drdy_excessive_latched = 0;
    Mag_RecordValidSample(h);

    return HAL_OK;
}

/* ── Telemetry period control ─────────────────────────────────────────────── */

static uint32_t mag_telemetry_period_ms = MAG_TELEMETRY_INTERVAL_MS;

void MAG_SetTelemetryPeriod(uint32_t ms)
{
    if (ms < 20U || ms > 5000U)
    {
        Logger_Log(LOG_INFO, "MAG_TELPER,ERR:RANGE,MIN:20,MAX:5000,OK:0");
        return;
    }
    mag_telemetry_period_ms = ms;
    Logger_Log(LOG_INFO, "MAG_TELPER,PERIOD_MS:%lu,OK:1", (unsigned long)ms);
}

uint32_t MAG_GetTelemetryPeriod(void)
{
    return mag_telemetry_period_ms;
}

/* ── Telemetry (rate-limited) ────────────────────────────────────────────── */

/* µT_x100 = raw * 10 / 3
 * raw=3000  -> 10000 -> 100.00 µT
 * raw=-3000 -> -10000 -> -100.00 µT */
static int32_t Mag_RawToUtX100(int16_t raw)
{
    return ((int32_t)raw * 10) / 3;
}

/* GUI-compatible telemetry.  Preserves exact key order expected by PySide6 GUI:
 * MX, MY, MZ, MX_UTX100, MY_UTX100, MZ_UTX100, BMAG_UTX100,
 * STATUS, DRDY, OVFL, AGE_MS, STATE, OK
 * Extended with health fields (backward-compatible — GUI ignores unknown keys):
 * INIT, FOUND, HAL_ST, HAL_ERR, COMM_ERR, DRDY_TOUT, DOR, RECOVERY         */
void MAG_QMC5883L_Telemetry(I2C_HandleTypeDef *hi2c)
{
    MAG_QMC5883L_Handle_t *h = &g_mag_handle;
    uint32_t now = HAL_GetTick();

    /* Rate-limit telemetry */
    if ((now - h->last_telemetry_tick) < mag_telemetry_period_ms)
        return;
    h->last_telemetry_tick = now;

    /* Determine OK:1 semantics:
     *   state == ONLINE
     *   has_valid_data
     *   fresh-sample age <= MAG_DATA_TIMEOUT_MS                     */
    uint8_t ok = 0;
    uint32_t age_ms = 0;
    int32_t mx_utx100 = 0, my_utx100 = 0, mz_utx100 = 0, bmag = 0;

    if (h->state == MAG_STATE_ONLINE && h->has_valid_data && h->initialized)
    {
        age_ms = (uint32_t)(now - h->last_fresh_sample_tick);
        if (age_ms <= MAG_DATA_TIMEOUT_MS)
        {
            mx_utx100 = Mag_RawToUtX100(h->last_valid_x);
            my_utx100 = Mag_RawToUtX100(h->last_valid_y);
            mz_utx100 = Mag_RawToUtX100(h->last_valid_z);

            int64_t sum_sq = (int64_t)mx_utx100 * mx_utx100 +
                             (int64_t)my_utx100 * my_utx100 +
                             (int64_t)mz_utx100 * mz_utx100;
            bmag = (int32_t)isqrt_u64((uint64_t)sum_sq);
            ok = 1;
        }
    }

    /* Compute sample age for non-ONLINE states too */
    if (!ok && h->has_valid_data)
    {
        age_ms = (uint32_t)(now - h->last_fresh_sample_tick);
    }

    uint8_t drdy = (h->last_valid_status & MAG_QMC5883L_STATUS_DRDY) ? 1U : 0U;
    uint8_t ovfl = (h->last_valid_status & MAG_QMC5883L_STATUS_OVFL) ? 1U : 0U;

    /* Always send telemetry — even when OK:0 — so GUI can show real state */
    Logger_Log(LOG_INFO,
               "MAG_IMU,MX:%d,MY:%d,MZ:%d,"
               "MX_UTX100:%ld,MY_UTX100:%ld,MZ_UTX100:%ld,"
               "BMAG_UTX100:%ld,"
               "STATUS:0x%02X,DRDY:%u,OVFL:%u,"
               "AGE_MS:%lu,STATE:%s,OK:%u,"
               "INIT:%u,FOUND:%u,"
               "HAL_ST:%d,HAL_ERR:%lu,"
               "COMM_ERR:%u,DRDY_TOUT:%u,DOR:%lu,RECOVERY:%lu,"
               "VERIFY_MM:%u",
               (int)h->last_valid_x, (int)h->last_valid_y, (int)h->last_valid_z,
               (long)mx_utx100, (long)my_utx100, (long)mz_utx100,
               (long)bmag,
               (unsigned)h->last_valid_status, drdy, ovfl,
               (unsigned long)age_ms, MagStateName(h->state), ok,
               h->initialized, h->found,
               (int)h->last_hal_status, (unsigned long)h->last_hal_error,
               (unsigned)h->consecutive_comm_errors,
               (unsigned)h->consecutive_drdy_timeouts,
               (unsigned long)h->total_dor_events,
               (unsigned long)h->recovery_count,
               (unsigned)h->verify_mismatch_count);
}

/* ── Full diagnostic status ──────────────────────────────────────────────── */

void MAG_QMC5883L_PrintStatus(I2C_HandleTypeDef *hi2c)
{
    MAG_QMC5883L_Handle_t *h = &g_mag_handle;

    Logger_Log(LOG_INFO,
               "MAG_STATUS,STATE:%s,FOUND:%u,INIT:%u,"
               "CHIP_ID:0x%02X,CTRL1_RB:0x%02X,SETRESET_RB:0x%02X",
               MagStateName(h->state), h->found, h->initialized,
               (unsigned)h->chip_id,
               (unsigned)h->ctrl1_readback, (unsigned)h->setreset_readback);

    /* Read live register values if online */
    if (h->found)
    {
        uint8_t ctrl1 = 0, ctrl2 = 0, sr = 0, status = 0;
        Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_CTRL1, &ctrl1);
        Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_CTRL2, &ctrl2);
        Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_SET_RESET, &sr);
        Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_STATUS, &status);
        Logger_Log(LOG_INFO,
                   "MAG_REGS_LIVE,CTRL1:0x%02X,CTRL2:0x%02X,"
                   "SETRESET:0x%02X,STATUS:0x%02X",
                   (unsigned)ctrl1, (unsigned)ctrl2,
                   (unsigned)sr, (unsigned)status);
    }

    /* Last valid measurement */
    if (h->has_valid_data)
    {
        int32_t mx = Mag_RawToUtX100(h->last_valid_x);
        int32_t my = Mag_RawToUtX100(h->last_valid_y);
        int32_t mz = Mag_RawToUtX100(h->last_valid_z);
        uint32_t age = (uint32_t)(HAL_GetTick() - h->last_fresh_sample_tick);
        Logger_Log(LOG_INFO,
                   "MAG_LAST_VALID,X:%d,Y:%d,Z:%d,"
                   "UTX100:%ld,%ld,%ld,AGE_MS:%lu",
                   (int)h->last_valid_x, (int)h->last_valid_y, (int)h->last_valid_z,
                   (long)mx, (long)my, (long)mz, (unsigned long)age);
    }

    /* Error counters */
    Logger_Log(LOG_INFO,
               "MAG_ERRORS,COMM:%lu,DRDY:%lu,INVALID:%lu,OVERFLOW:%lu,DOR:%lu",
               (unsigned long)h->total_comm_errors,
               (unsigned long)h->total_drdy_timeouts,
               (unsigned long)h->total_invalid_samples,
               (unsigned long)h->total_overflows,
               (unsigned long)h->total_dor_events);

    Logger_Log(LOG_INFO,
               "MAG_CONSEC,COMM:%u,DRDY:%u,INVALID:%u,VALID_STREAK:%u,IDENTICAL:%lu",
               (unsigned)h->consecutive_comm_errors,
               (unsigned)h->consecutive_drdy_timeouts,
               (unsigned)h->consecutive_invalid_samples,
               (unsigned)h->valid_sample_streak,
               (unsigned long)h->identical_sample_count);

    Logger_Log(LOG_INFO,
               "MAG_VERIFY,MISMATCH_COUNT:%u,ATTEMPTS:%u",
               (unsigned)h->verify_mismatch_count,
               (unsigned)h->verify_attempts);

    Logger_Log(LOG_INFO,
               "MAG_RECONNECTS:%lu,RECOVERIES:%lu,HAL_STATUS:%d,HAL_ERR:%lu",
               (unsigned long)h->reconnect_count,
               (unsigned long)h->recovery_count,
               (int)h->last_hal_status,
               (unsigned long)h->last_hal_error);

    /* Bus line status */
    GPIO_PinState sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    GPIO_PinState scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    Logger_Log(LOG_INFO,
               "MAG_BUS,SDA:%u,SCL:%u,I2C_STATE:%lu",
               (sda == GPIO_PIN_SET) ? 1U : 0U,
               (scl == GPIO_PIN_SET) ? 1U : 0U,
               (unsigned long)HAL_I2C_GetState(hi2c));
}

/* ── Legacy API: Detect ──────────────────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_Detect(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag)
{
    return Mag_DetectInternal(hi2c, mag);
}

/* ── Legacy API: Init ────────────────────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_Init(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag)
{
    /* Legacy blocking init — use only from commands, not from the main loop.
     * Uses HAL_Delay intentionally for backward compatibility with maginit. */
    if (!mag->found)
    {
        HAL_StatusTypeDef st = Mag_DetectInternal(hi2c, mag);
        if (st != HAL_OK)
            return st;
    }

#if MAG_STRICT_CHIP_ID_CHECK
    if (mag->chip_id != MAG_QMC5883L_CHIP_ID_EXPECTED)
        return HAL_ERROR;
#endif

    HAL_StatusTypeDef st;
    st = Mag_WriteReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_CTRL2, 0x80);
    if (st != HAL_OK) return st;
    HAL_Delay(20);

    st = Mag_WriteReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_SET_RESET, 0x01);
    if (st != HAL_OK) return st;
    HAL_Delay(10);

    st = Mag_WriteReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_CTRL1,
                      MAG_QMC5883L_CTRL1_VALUE);
    if (st != HAL_OK) return st;
    HAL_Delay(20);

    uint8_t ctrl1_rb = 0, setreset_rb = 0;
    st = Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_CTRL1, &ctrl1_rb);
    if (st != HAL_OK) return st;
    st = Mag_ReadReg(hi2c, MAG_QMC5883L_ADDR7, MAG_QMC5883L_REG_SET_RESET, &setreset_rb);
    if (st != HAL_OK) return st;

    if (ctrl1_rb != MAG_QMC5883L_CTRL1_VALUE || setreset_rb != 0x01)
        return HAL_ERROR;

    mag->initialized = 1;
    return HAL_OK;
}

/* ── Legacy API: ReadRaw ─────────────────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_ReadRaw(I2C_HandleTypeDef *hi2c,
                                       MAG_QMC5883P_Handle_t *mag,
                                       MAG_QMC5883P_Raw_t *raw)
{
    MAG_QMC5883L_Handle_t *h = &g_mag_handle;

    /* Use the state machine's last valid data */
    if (!h->has_valid_data || !h->initialized)
        return HAL_ERROR;

    raw->x      = h->last_valid_x;
    raw->y      = h->last_valid_y;
    raw->z      = h->last_valid_z;
    raw->status = 0x01;  /* DRDY implied */
    return HAL_OK;
}

/* ── Legacy API: ReadImu ─────────────────────────────────────────────────── */

HAL_StatusTypeDef MAG_QMC5883P_ReadImu(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag)
{
    MAG_QMC5883L_Telemetry(hi2c);
    return g_mag_handle.has_valid_data ? HAL_OK : HAL_ERROR;
}
