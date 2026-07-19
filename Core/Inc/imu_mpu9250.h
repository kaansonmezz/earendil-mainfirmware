#ifndef IMU_MPU9250_H
#define IMU_MPU9250_H

#include "stm32h7xx_hal.h"

/* ── I2C address ─────────────────────────────────────────────────────────── */
#define MPU9250_ADDR_7BIT       0x68U
#define MPU9250_ADDR_HAL        (MPU9250_ADDR_7BIT << 1)  /* 0xD0 */

/* ── Register map (common to MPU6500 / MPU9250 accel+gyro block) ────────── */
#define MPU9250_REG_SMPLRT_DIV  0x19U
#define MPU9250_REG_CONFIG      0x1AU
#define MPU9250_REG_GYRO_CONFIG 0x1BU
#define MPU9250_REG_ACCEL_CONFIG    0x1CU
#define MPU9250_REG_ACCEL_CONFIG2   0x1DU
#define MPU9250_REG_INT_PIN_CFG 0x37U
#define MPU9250_REG_USER_CTRL   0x6AU
#define MPU9250_REG_PWR_MGMT_1  0x6BU
#define MPU9250_REG_PWR_MGMT_2  0x6CU
#define MPU9250_REG_WHO_AM_I    0x75U
#define MPU9250_REG_ACCEL_XOUT_H 0x3BU
#define MPU9250_REG_GYRO_XOUT_H  0x43U
#define MPU9250_REG_INT_STATUS   0x3AU

/* ── Raw sensor data ─────────────────────────────────────────────────────── */
typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_MPU9250_Raw_t;

/* ── Converted sensor data (scaled integer) ──────────────────────────────── */
typedef struct
{
    int32_t acc_x_mg;       /* milli-g  (1000 mg = 1 g)  */
    int32_t acc_y_mg;
    int32_t acc_z_mg;
    int32_t temp_cx100;     /* centi-degC (100 = 1.00 C) */
    int32_t gyro_x_mdps;    /* milli-dps (1000 mdps = 1 dps) */
    int32_t gyro_y_mdps;
    int32_t gyro_z_mdps;
} IMU_MPU9250_Conv_t;

/* ── Public API ──────────────────────────────────────────────────────────── */

/* Returns true for supported WHO_AM_I values: 0x70, 0x71, 0x73. */
uint8_t IMU_MPU9250_IsSupportedWho(uint8_t who);

/* Low-level register helpers (polling I2C, 50 ms timeout). */
HAL_StatusTypeDef IMU_MPU9250_WriteReg(I2C_HandleTypeDef *hi2c,
                                       uint8_t reg, uint8_t value);
HAL_StatusTypeDef IMU_MPU9250_ReadReg(I2C_HandleTypeDef *hi2c,
                                      uint8_t reg, uint8_t *value);

/* Scan 0x03..0x68 and write register immediately when 0x68 ACKs. */
HAL_StatusTypeDef IMU_MPU9250_FindAndWriteReg(I2C_HandleTypeDef *hi2c,
                                               uint8_t reg, uint8_t value);

/* Scan 0x03..0x68 and read register immediately when 0x68 ACKs. */
HAL_StatusTypeDef IMU_MPU9250_FindAndReadReg(I2C_HandleTypeDef *hi2c,
                                              uint8_t reg, uint8_t *value);

/* Stage 3: WHO_AM_I probe. */
void IMU_MPU9250_WhoAmI(I2C_HandleTypeDef *hi2c);

/* Diagnostic: probe before/after warm-up only, no WHO read. */
void IMU_MPU9250_WarmupProbe(I2C_HandleTypeDef *hi2c);

/* Scan 0x03..0x68 and read WHO_AM_I immediately when 0x68 responds.
 * Returns HAL_OK if WHO_AM_I was read (check *who_out for supported value). */
HAL_StatusTypeDef IMU_MPU9250_FindAndReadWho(I2C_HandleTypeDef *hi2c,
                                              uint8_t *who_out);

/* Stage 4: basic init (reset, clock, accel/gyro config). */
HAL_StatusTypeDef IMU_MPU9250_InitBasic(I2C_HandleTypeDef *hi2c);

/* Diagnostic: CONFIG register write/readback test. */
void IMU_MPU9250_CfgTest(I2C_HandleTypeDef *hi2c);

/* Scan 0x03..0x68 and burst-read `len` bytes starting at `start_reg`. */
HAL_StatusTypeDef IMU_MPU9250_FindAndReadBytes(I2C_HandleTypeDef *hi2c,
                                               uint8_t start_reg,
                                               uint8_t *buf, uint16_t len);

/* Verbose variant: logs MPU_FINDBURST for diagnostic commands (mpugyrotest). */
HAL_StatusTypeDef IMU_MPU9250_FindAndReadBytesVerbose(I2C_HandleTypeDef *hi2c,
                                                      uint8_t start_reg,
                                                      uint8_t *buf, uint16_t len);

/* One-shot raw accel/gyro/temperature read (14 bytes from 0x3B). */
HAL_StatusTypeDef IMU_MPU9250_ReadRaw(I2C_HandleTypeDef *hi2c,
                                      IMU_MPU9250_Raw_t *raw);

/* Update CubeIDE Live Expression debug variables from a raw read result. */
void IMU_MPU9250_UpdateDebugRaw(const IMU_MPU9250_Raw_t *raw, uint8_t ok);

/* Stage 5: gyro-specific diagnostic (clock source, config, raw registers). */
void IMU_MPU9250_GyroTest(I2C_HandleTypeDef *hi2c);

/* Stage 6: read and convert raw accel/gyro/temp to physical units. */
HAL_StatusTypeDef IMU_MPU9250_ReadConverted(I2C_HandleTypeDef *hi2c,
                                            IMU_MPU9250_Conv_t *conv);

/* Gyro bias control API. */
void    IMU_MPU9250_BiasQuery(void);
void    IMU_MPU9250_BiasEnable(void);
void    IMU_MPU9250_BiasDisable(void);
void    IMU_MPU9250_BiasClear(void);
uint8_t IMU_MPU9250_BiasIsEnabled(void);
uint8_t IMU_MPU9250_BiasGetSource(void);
int16_t IMU_MPU9250_BiasGetX(void);
int16_t IMU_MPU9250_BiasGetY(void);
int16_t IMU_MPU9250_BiasGetZ(void);

/* IMU stream control API. */
void     IMU_StreamOn(void);
void     IMU_StreamOff(void);
void     IMU_StreamSetPeriod(uint32_t ms);
uint32_t IMU_StreamGetPeriod(void);
uint8_t  IMU_StreamIsEnabled(void);
void     IMU_StreamTask(void);

/* Per-sensor telemetry period control. */
void     IMU_GyroSetTelemetryPeriod(uint32_t ms);
uint32_t IMU_GyroGetTelemetryPeriod(void);
void     IMU_AccelSetTelemetryPeriod(uint32_t ms);
uint32_t IMU_AccelGetTelemetryPeriod(void);

/* Gyro output filter API (LPF + deadband, display-only). */
void     IMU_GyroFilterOn(void);
void     IMU_GyroFilterOff(void);
void     IMU_GyroFilterStatus(void);
void     IMU_GyroFilterSetDeadband(int32_t mdps);
void     IMU_GyroFilterSetLpfAlpha(int32_t alpha_permille);
uint8_t  IMU_GyroFilterIsEnabled(void);
int32_t  IMU_GyroFilterGetDeadband(void);
int32_t  IMU_GyroFilterGetLpfAlpha(void);
void     IMU_ApplyGyroFilter(int32_t *gx_mdps, int32_t *gy_mdps, int32_t *gz_mdps);

/* ── CubeIDE Live Expression debug variables (extern) ──────────────────────
 * Defined in imu_mpu9250.c as volatile with __attribute__((used)).
 * Add these to CubeIDE Live Expressions to watch raw IMU values. */
extern volatile int16_t  imu_dbg_acc_x;
extern volatile int16_t  imu_dbg_acc_y;
extern volatile int16_t  imu_dbg_acc_z;
extern volatile int16_t  imu_dbg_temp;
extern volatile int16_t  imu_dbg_gyro_x;
extern volatile int16_t  imu_dbg_gyro_y;
extern volatile int16_t  imu_dbg_gyro_z;
extern volatile uint8_t  imu_dbg_raw_ok;
extern volatile uint32_t imu_dbg_raw_counter;
extern volatile uint32_t imu_dbg_raw_fail_counter;

#endif /* IMU_MPU9250_H */
