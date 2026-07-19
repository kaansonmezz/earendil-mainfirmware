#ifndef MAG_QMC5883P_H
#define MAG_QMC5883P_H

#include "stm32h7xx_hal.h"

/* I2C1 timing constant moved to app_config.h (I2C_TIMING_APP).
 * See app_config.h for field decode and verification notes. */

/* ── Sensor identity: QMC5883L ─────────────────────────────────────────────
 * Filename intentionally retained as mag_qmc5883p.* to avoid cascading
 * Makefile / include changes.  The legacy suffix "p" in filenames is a
 * historical artifact — the physical sensor is QMC5883L (addr 0x0D,
 * chip ID 0xFF).  All symbols, constants, register definitions, and
 * behavior below are for the QMC5883L sensor (or register-compatible
 * clones).  Public MAG_QMC5883P_* symbols are backward-compatibility
 * aliases that map to the corrected MAG_QMC5883L_* names. */

/* ── I2C address ─────────────────────────────────────────────────────────── */
#define MAG_QMC5883L_ADDR7              0x0DU
#define MAG_QMC5883L_DEVADDR_HAL        (MAG_QMC5883L_ADDR7 << 1)  /* 0x1A */

/* ── Register map (QMC5883L) ─────────────────────────────────────────────── */
#define MAG_QMC5883L_REG_X_LSB          0x00U
#define MAG_QMC5883L_REG_X_MSB          0x01U
#define MAG_QMC5883L_REG_Y_LSB          0x02U
#define MAG_QMC5883L_REG_Y_MSB          0x03U
#define MAG_QMC5883L_REG_Z_LSB          0x04U
#define MAG_QMC5883L_REG_Z_MSB          0x05U
#define MAG_QMC5883L_REG_STATUS         0x06U
#define MAG_QMC5883L_REG_CTRL1          0x09U
#define MAG_QMC5883L_REG_CTRL2          0x0AU
#define MAG_QMC5883L_REG_SET_RESET      0x0BU
#define MAG_QMC5883L_REG_CHIP_ID        0x0DU

#define MAG_QMC5883L_CHIP_ID_EXPECTED   0xFFU

/* ── Status register bits ────────────────────────────────────────────────── */
#define MAG_QMC5883L_STATUS_DRDY        (1U << 0)
#define MAG_QMC5883L_STATUS_OVFL        (1U << 1)
#define MAG_QMC5883L_STATUS_DOR         (1U << 2) /* Data overrun (skipped) */

/* ── CTRL1 value: OSR=512, Range=±8G, ODR=200Hz, Mode=continuous ──────────
 *   0x1D = 0b00011101
 *   Bits [7:6] = 00  -> OSR:   512
 *   Bits [5:4] = 01  -> Range: ±8 Gauss
 *   Bits [3:2] = 11  -> ODR:   200 Hz
 *   Bits [1:0] = 01  -> Mode:  continuous                               */
#define MAG_QMC5883L_CTRL1_VALUE        0x1DU

/* ── Conversion: ±8 Gauss, 3000 LSB/Gauss ───────────────────────────────
 *   µT = raw / 30   (1 G = 100 µT, so raw * 100 / 3000 = raw / 30)
 *   µT_x100 = raw * 10 / 3                                              */

/* ── Chip-ID policy ─────────────────────────────────────────────────────
 * Set to 1 to reject any chip ID other than 0xFF.
 * Set to 0 to accept any ACKing device at 0x0D (for clones).            */
#define MAG_STRICT_CHIP_ID_CHECK        1U

/* ── Configurable thresholds ─────────────────────────────────────────────── */
#define MAG_ACQUIRE_INTERVAL_MS             20U   /* 50 Hz acquisition */
#define MAG_RECONNECT_INTERVAL_MS          500U
#define MAG_I2C_TIMEOUT_MS                   5U
#define MAG_MAX_CONSECUTIVE_COMM_ERRORS      5U
#define MAG_MAX_CONSECUTIVE_DRDY_TIMEOUTS    5U
#define MAG_MAX_CONSECUTIVE_INVALID         10U
#define MAG_REQUIRED_VALID_SAMPLES           5U
#define MAG_DATA_TIMEOUT_MS                500U   /* no fresh DRDY-backed sample for this long → sensor stuck */
#define MAG_TELEMETRY_INTERVAL_MS          100U   /* 10 Hz telemetry */
#define MAG_MAX_VERIFY_ATTEMPTS              3U   /* VERIFY readback failures before recovery */
#define MAG_MAX_PROBE_FAILURES               3U   /* consecutive probe failures before bus recovery */
#define MAG_RECOVERY_RETRY_INTERVAL_MS    2000U   /* wait after failed recovery before retry */
#define MAG_INIT_TIMEOUT_MS               5000U   /* max time for entire init sequence */

/* ── State machine ───────────────────────────────────────────────────────── */
typedef enum
{
    MAG_STATE_OFFLINE = 0,
    MAG_STATE_RECOVERING,
    MAG_STATE_PROBING,
    MAG_STATE_RESETTING,
    MAG_STATE_WAIT_RESET,
    MAG_STATE_SETRESET,
    MAG_STATE_WAIT_SETRESET,
    MAG_STATE_CTRL1,
    MAG_STATE_WAIT_CTRL1,
    MAG_STATE_VERIFY,
    MAG_STATE_STABILIZING,
    MAG_STATE_ONLINE,
    MAG_STATE_FAULT
} MagState_t;

/* ── Raw sensor data ─────────────────────────────────────────────────────── */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t status;
} MAG_QMC5883L_Raw_t;

/* ── State handle ────────────────────────────────────────────────────────── */
typedef struct
{
    /* Identity */
    uint8_t  found;
    uint8_t  initialized;
    uint8_t  addr7;
    uint8_t  chip_id;

    /* State machine */
    MagState_t state;
    uint32_t state_tick;                /* tick when current state was entered */

    /* Timing */
    uint32_t last_reconnect_tick;
    uint32_t last_success_tick;
    uint32_t last_telemetry_tick;
    uint32_t last_acquire_tick;
    uint32_t deadline_tick;             /* sub-state deadline for non-blocking init */

    /* Validated measurement */
    int16_t  last_valid_x;
    int16_t  last_valid_y;
    int16_t  last_valid_z;
    uint8_t  last_valid_status;
    uint32_t last_valid_tick;
    uint8_t  has_valid_data;

    /* Fresh-sample timestamp (updated on every DRDY-backed valid read) */
    uint32_t last_fresh_sample_tick;
    uint32_t identical_sample_count;    /* diagnostic only — no state effect */

    /* Error counters */
    uint32_t total_comm_errors;
    uint32_t total_drdy_timeouts;
    uint32_t total_invalid_samples;
    uint32_t total_overflows;
    uint32_t total_dor_events;
    uint32_t reconnect_count;
    uint32_t recovery_count;
    uint16_t consecutive_comm_errors;
    uint16_t consecutive_drdy_timeouts;
    uint16_t consecutive_invalid_samples;
    uint8_t  valid_sample_streak;

    /* Probe / verify sub-state counters */
    uint8_t  probe_failure_count;
    uint8_t  verify_attempts;
    uint8_t  verify_mismatch_count;     /* readback value mismatch (not I2C failure) */

    /* Last HAL error info */
    HAL_StatusTypeDef last_hal_status;
    uint32_t last_hal_error;

    /* Init sub-state (for non-blocking init sequence) */
    uint8_t  init_step;

    /* Init deadline — overall timeout for the entire init sequence */
    uint32_t init_deadline_tick;

    /* Register readback verification */
    uint8_t  ctrl1_readback;
    uint8_t  setreset_readback;

    /* Recovery tracking */
    uint8_t  recovery_pending;

    /* DRDY excessive timeout latch — prevents log spam.
     * Set when consecutive_drdy_timeouts first crosses MAG_MAX_CONSECUTIVE_DRDY_TIMEOUTS.
     * Cleared when a valid DRDY-backed sample arrives. */
    uint8_t  drdy_excessive_latched;
} MAG_QMC5883L_Handle_t;

/* ── Backward compatibility aliases ──────────────────────────────────────── */
#define MAG_QMC5883P_ADDR7              MAG_QMC5883L_ADDR7
#define MAG_QMC5883P_DEVADDR_HAL        MAG_QMC5883L_DEVADDR_HAL
#define MAG_QMC5883P_CHIP_ID_EXPECTED   MAG_QMC5883L_CHIP_ID_EXPECTED
#define MAG_QMC5883P_STATUS_DRDY        MAG_QMC5883L_STATUS_DRDY
#define MAG_QMC5883P_STATUS_OVFL        MAG_QMC5883L_STATUS_OVFL
#define MAG_QMC5883P_REG_CHIP_ID        MAG_QMC5883L_REG_CHIP_ID
#define MAG_QMC5883P_REG_SET_RESET      MAG_QMC5883L_REG_SET_RESET
#define MAG_QMC5883P_REG_CTRL1          MAG_QMC5883L_REG_CTRL1
#define MAG_QMC5883P_REG_CTRL2          MAG_QMC5883L_REG_CTRL2

typedef MAG_QMC5883L_Raw_t    MAG_QMC5883P_Raw_t;
typedef MAG_QMC5883L_Handle_t MAG_QMC5883P_Handle_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

extern MAG_QMC5883L_Handle_t g_mag_handle;

/* Full runtime reset of magnetometer driver state.
 * Preserves lifetime diagnostic counters when preserve_diagnostics=1.
 * Resets all transient state: consecutive error counters, init sub-state,
 * cached samples, timestamps, probe/verify counters, recovery flags. */
void MAG_QMC5883L_RuntimeReset(uint8_t preserve_diagnostics);

/* State machine: call periodically from the main loop.
 * Non-blocking.  Manages probing, reset, configuration, and recovery. */
void MAG_QMC5883L_Task(I2C_HandleTypeDef *hi2c);

/* Acquire one sample if DRDY is ready.  Non-blocking.
 * Returns HAL_OK if a valid sample was read and stored. */
HAL_StatusTypeDef MAG_QMC5883L_Acquire(I2C_HandleTypeDef *hi2c);

/* Publish telemetry at the configured telemetry rate.
 * Separated from acquisition to avoid blocking the main loop. */
void MAG_QMC5883L_Telemetry(I2C_HandleTypeDef *hi2c);

/* Set/get magnetometer telemetry period (ms).  Range: 20..5000. */
void     MAG_SetTelemetryPeriod(uint32_t ms);
uint32_t MAG_GetTelemetryPeriod(void);

/* Full diagnostic status report for the 'magstatus' command. */
void MAG_QMC5883L_PrintStatus(I2C_HandleTypeDef *hi2c);

/* Legacy API wrappers (map to new names) */
HAL_StatusTypeDef MAG_QMC5883P_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef MAG_QMC5883P_Detect(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag);
HAL_StatusTypeDef MAG_QMC5883P_Init(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag);
HAL_StatusTypeDef MAG_QMC5883P_ReadRaw(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag, MAG_QMC5883P_Raw_t *raw);
HAL_StatusTypeDef MAG_QMC5883P_ReadImu(I2C_HandleTypeDef *hi2c, MAG_QMC5883P_Handle_t *mag);

#endif /* MAG_QMC5883P_H */
