# STM32H723ZG Rover IMU Integration Roadmap

## 0. Purpose

This roadmap defines how to add a clean, fault-tolerant IMU system to the STM32H723ZG rover main-controller firmware and integrate it with the PySide6 GUI file `earendil.py`.

The current firmware project should be treated as the stable rover-control base. The IMU must be added without breaking existing terminal commands, operating-mode behavior, motor UART communication, motor telemetry parsing, safety logic, or GUI controls.

The IMU hardware is a GY-91-style module with an MPU9250-marked chip, but the magnetometer is not available/usable. Therefore, the first implementation must treat it as a 6-axis IMU:

```text
Accel X/Y/Z
Gyro X/Y/Z
```

Future magnetometer support should be planned with placeholders, but it must not be falsely implemented in the first stage.

---

## 1. Critical Rules

### 1.1 Do not break existing rover behavior

The following existing firmware commands must keep working exactly as before:

```text
help
status
termstat
mode
mode disarm
mode manual
mode auto
mode autonomous
m speed
m duty
stop
brake
identify
f0..f200
b0..b200
r0..r200
l0..l200
fd0..fd4000
bd0..bd4000
rd0..rd4000
ld0..ld4000
FL <raw command>
FR <raw command>
RL <raw command>
RR <raw command>
ALL <tuning command>
```

The GUI operating-mode indicator in `earendil.py` relies on H7 confirmation logs containing these substrings:

```text
[MODE] DISARM active
[MODE] MANUAL active
[MODE] AUTONOMOUS active
```

Do not change or remove those confirmation messages.

### 1.2 IMU must never block the rover

The IMU must not be able to prevent the rover firmware from booting or responding to commands.

Bad behavior:

```c
while (MPU_Init() != OK)
{
    // wait forever
}
```

Bad behavior:

```c
while (1)
{
    LongBlockingImuRead();
    App_Update();
}
```

Required behavior:

```c
while (1)
{
    App_Update();
    ImuManager_Update();
}
```

`App_Update()` must remain the highest-priority periodic application call.

### 1.3 No continuous IMU terminal spam by default

After boot, IMU streaming must be OFF by default.

The firmware must not continuously print IMU values unless the user explicitly enables streaming with:

```text
imu stream on
```

---

## 2. Hardware Definition

### 2.1 Module pins

The module has these pins:

```text
VIN
3V3
GND
SCL
SDA
SDO/SA0   // single pin
NCS
CSB
```

### 2.2 Required wiring for I2C mode

Use I2C1 on the STM32H723ZG board.

```text
GY-91 3V3      -> STM32 3V3
GY-91 VIN      -> leave unconnected when using 3V3
GY-91 GND      -> STM32 GND
GY-91 SCL      -> PB8 / I2C1_SCL
GY-91 SDA      -> PB9 / I2C1_SDA
GY-91 SDO/SA0  -> GND
GY-91 NCS      -> 3V3
GY-91 CSB      -> 3V3 or keep inactive/high if required by the module
```

### 2.3 I2C address

With `SDO/SA0 -> GND`, the 7-bit device address is:

```c
0x68
```

STM32 HAL APIs expect the shifted 8-bit address:

```c
#define IMU_I2C_ADDR  (0x68U << 1)  // 0xD0
```

If `SDO/SA0` is tied to 3V3 in the future, the 7-bit address becomes `0x69`, so the HAL address becomes:

```c
#define IMU_I2C_ADDR  (0x69U << 1)  // 0xD2
```

### 2.4 Pull-ups

I2C requires pull-up resistors on SDA/SCL. If the module does not already include pull-ups, add:

```text
SDA -> 4.7kΩ -> 3V3
SCL -> 4.7kΩ -> 3V3
```

---

## 3. Existing Project Architecture to Preserve

The firmware already uses a clean high-level application structure:

```c
App_Init();

while (1)
{
    App_Update();
}
```

Keep this structure. Add IMU as a secondary module after the existing rover application update:

```c
App_Init();
ImuManager_Init(&hi2c1);

while (1)
{
    App_Update();
    ImuManager_Update();
}
```

Do not move existing motor, safety, UART, operating-mode, or terminal logic into the IMU module.

---

## 4. File-Level Implementation Plan

### 4.1 Firmware files to add

Add these new files:

```text
Core/Inc/imu_mpu6axis.h
Core/Src/imu_mpu6axis.c

Core/Inc/imu_manager.h
Core/Src/imu_manager.c
```

### 4.2 Firmware files likely to modify

Modify these files only as needed:

```text
Core/Src/main.c
Core/Src/stm32h7xx_hal_msp.c
Core/Inc/stm32h7xx_hal_conf.h
Core/Inc/terminal_parser.h
Core/Src/terminal_parser.c
Core/Src/command_handler.c
Core/Inc/command_handler.h    // only if required by existing style
Core/Inc/logger.h             // only if adding a raw-line logger helper
Core/Src/logger.c             // only if adding a raw-line logger helper
```

### 4.3 GUI file to modify

Modify the GUI file:

```text
earendil.py
```

Do not refer to the GUI by any other filename.

---

## 5. Stage 1 — Add I2C1 Infrastructure

### 5.1 `main.c`

Add the I2C handle:

```c
I2C_HandleTypeDef hi2c1;
```

Add the init prototype near other `MX_*_Init()` prototypes:

```c
static void MX_I2C1_Init(void);
```

Call I2C init before `App_Init()` and before `ImuManager_Init()`:

```c
MX_I2C1_Init();

App_Init();
ImuManager_Init(&hi2c1);
```

Add the IMU update call after `App_Update()`:

```c
while (1)
{
    App_Update();
    ImuManager_Update();
}
```

### 5.2 `MX_I2C1_Init()` target configuration

Use a normal 100 kHz or 400 kHz I2C configuration. Start with conservative 100 kHz if signal integrity is uncertain.

The exact STM32H7 timing value should match Cube/HAL timing conventions already used in the project. Do not guess blindly if the project has an existing clock-tree style. If unsure, use STM32CubeMX-generated timing for I2C1 with the project clock tree.

### 5.3 `stm32h7xx_hal_msp.c`

Add `HAL_I2C_MspInit()` for I2C1.

Required GPIO setup:

```c
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_I2C1_CLK_ENABLE();

GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
```

If the project uses `HAL_I2C_MspDeInit()`, add a matching deinit implementation.

### 5.4 `stm32h7xx_hal_conf.h`

Ensure this is enabled:

```c
#define HAL_I2C_MODULE_ENABLED
```

Do not disable existing UART, DMA, GPIO, or timer modules.

---

## 6. Stage 2 — Low-Level 6-Axis MPU Driver

### 6.1 Goal

Implement a small MPU driver that only talks to the physical sensor. It must not know about rover operating modes, terminal commands, GUI streaming, motor UARTs, or safety logic.

### 6.2 Driver name

Use:

```text
imu_mpu6axis.c
imu_mpu6axis.h
```

This is intentional. The sensor may be marked MPU9250, but magnetometer data is unavailable, so the first implementation is 6-axis only.

### 6.3 Required registers

Define at least:

```c
#define IMU_MPU_ADDR              (0x68U << 1)

#define IMU_MPU_REG_SMPLRT_DIV    0x19U
#define IMU_MPU_REG_CONFIG        0x1AU
#define IMU_MPU_REG_GYRO_CONFIG   0x1BU
#define IMU_MPU_REG_ACCEL_CONFIG  0x1CU
#define IMU_MPU_REG_ACCEL_CONFIG2 0x1DU

#define IMU_MPU_REG_ACCEL_XOUT_H  0x3BU
#define IMU_MPU_REG_PWR_MGMT_1    0x6BU
#define IMU_MPU_REG_PWR_MGMT_2    0x6CU
#define IMU_MPU_REG_USER_CTRL     0x6AU
#define IMU_MPU_REG_WHO_AM_I      0x75U
```

### 6.4 Supported WHO_AM_I values

Accept these values:

```text
0x71  MPU9250-like
0x73  MPU9250/MPU9255-like variant
0x70  MPU6500-like
0x68  MPU6050-like
```

If any other value is read, report it through IMU status but do not crash the firmware.

### 6.5 Data structure

Create a structure similar to this:

```c
typedef struct
{
    int16_t raw_ax;
    int16_t raw_ay;
    int16_t raw_az;

    int16_t raw_gx;
    int16_t raw_gy;
    int16_t raw_gz;

    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float gyro_x_offset_dps;
    float gyro_y_offset_dps;
    float gyro_z_offset_dps;

    uint8_t who_am_i;

    bool present;
    bool initialized;
    bool healthy;

    uint32_t fail_count;
} ImuMpu6Axis_t;
```

Do not add fake magnetometer fields here. Magnetometer support will be a future separate stage.

### 6.6 Driver API

Expose this API:

```c
bool ImuMpu6Axis_ReadWhoAmI(I2C_HandleTypeDef *hi2c, uint8_t *who);

bool ImuMpu6Axis_Init(I2C_HandleTypeDef *hi2c, ImuMpu6Axis_t *imu);

bool ImuMpu6Axis_ReadAccelGyro(I2C_HandleTypeDef *hi2c, ImuMpu6Axis_t *imu);

bool ImuMpu6Axis_CalibrateGyro(I2C_HandleTypeDef *hi2c,
                               ImuMpu6Axis_t *imu,
                               uint16_t samples);
```

Optional helper:

```c
void ImuMpu6Axis_Clear(ImuMpu6Axis_t *imu);
```

### 6.7 Sensor init sequence

Use this simple init sequence:

```text
PWR_MGMT_1    = 0x00   wake up
small delay
PWR_MGMT_1    = 0x01   use gyro PLL clock
PWR_MGMT_2    = 0x00   enable all accel/gyro axes
CONFIG        = 0x03   DLPF setting
SMPLRT_DIV    = 0x09   roughly 100 Hz class output rate
GYRO_CONFIG   = 0x08   +/-500 dps
ACCEL_CONFIG  = 0x00   +/-2g
ACCEL_CONFIG2 = 0x03   accel DLPF
USER_CTRL     = 0x00   disable internal I2C master / no magnetometer bridge
```

Use short I2C timeouts, for example 5 ms or 10 ms. Do not use repeated 100 ms blocking timeouts in periodic IMU updates.

### 6.8 Scale factors

With the selected ranges:

```c
#define IMU_ACCEL_SCALE_LSB_PER_G     16384.0f
#define IMU_GYRO_SCALE_LSB_PER_DPS    65.5f
```

Conversion:

```c
imu->ax_g = (float)imu->raw_ax / IMU_ACCEL_SCALE_LSB_PER_G;
imu->ay_g = (float)imu->raw_ay / IMU_ACCEL_SCALE_LSB_PER_G;
imu->az_g = (float)imu->raw_az / IMU_ACCEL_SCALE_LSB_PER_G;

imu->gx_dps = ((float)imu->raw_gx / IMU_GYRO_SCALE_LSB_PER_DPS) - imu->gyro_x_offset_dps;
imu->gy_dps = ((float)imu->raw_gy / IMU_GYRO_SCALE_LSB_PER_DPS) - imu->gyro_y_offset_dps;
imu->gz_dps = ((float)imu->raw_gz / IMU_GYRO_SCALE_LSB_PER_DPS) - imu->gyro_z_offset_dps;
```

### 6.9 14-byte burst read

Read 14 bytes from `ACCEL_XOUT_H`:

```text
0x3B AX_H
0x3C AX_L
0x3D AY_H
0x3E AY_L
0x3F AZ_H
0x40 AZ_L
0x41 TEMP_H
0x42 TEMP_L
0x43 GX_H
0x44 GX_L
0x45 GY_H
0x46 GY_L
0x47 GZ_H
0x48 GZ_L
```

The temperature bytes can be ignored in the first implementation.

---

## 7. Stage 3 — IMU Manager

### 7.1 Goal

`imu_manager.c` integrates the IMU into the rover application without letting IMU failures affect the rover control system.

### 7.2 Manager API

Create:

```c
void ImuManager_Init(I2C_HandleTypeDef *hi2c);
void ImuManager_Update(void);

bool ImuManager_IsPresent(void);
bool ImuManager_IsHealthy(void);
bool ImuManager_IsStreamEnabled(void);

void ImuManager_PrintStatus(void);
void ImuManager_PrintOnce(void);
void ImuManager_SetStreamEnabled(bool enabled);
void ImuManager_CalibrateGyro(void);
```

### 7.3 Internal state

Use static internal state:

```c
static I2C_HandleTypeDef *s_hi2c;
static ImuMpu6Axis_t s_imu;

static bool s_present;
static bool s_initialized;
static bool s_healthy;
static bool s_stream_enabled;

static uint32_t s_last_sample_ms;
static uint32_t s_last_stream_ms;
static uint32_t s_last_retry_ms;
static uint32_t s_last_status_error_ms;
static uint32_t s_consecutive_fail_count;
```

### 7.4 Update behavior

`ImuManager_Update()` must be time-based and non-blocking.

No `HAL_Delay()` inside `ImuManager_Update()`.

Target logic:

```c
void ImuManager_Update(void)
{
    uint32_t now = HAL_GetTick();

    if (!s_initialized)
    {
        if ((now - s_last_retry_ms) >= 2000U)
        {
            ImuManager_TryInitOnce();
            s_last_retry_ms = now;
        }
        return;
    }

    if ((now - s_last_sample_ms) >= 10U)
    {
        bool ok = ImuMpu6Axis_ReadAccelGyro(s_hi2c, &s_imu);
        if (ok)
        {
            s_healthy = true;
            s_consecutive_fail_count = 0U;
        }
        else
        {
            s_consecutive_fail_count++;
            s_imu.fail_count++;
            if (s_consecutive_fail_count >= 3U)
            {
                s_healthy = false;
            }
        }
        s_last_sample_ms = now;
    }

    if (s_stream_enabled && ((now - s_last_stream_ms) >= 100U))
    {
        ImuManager_PrintTelemetryRaw();
        s_last_stream_ms = now;
    }
}
```

### 7.5 Failure behavior

If the IMU is missing or disconnected:

```text
Firmware must still boot.
App_Init must still run.
App_Update must still run.
Terminal commands must still work.
Motor control must still work.
Operating mode must still work.
No Error_Handler.
No infinite retry loop.
No terminal spam.
```

Retry sensor detection slowly, for example every 2000 ms.

---

## 8. Stage 4 — Terminal Commands

### 8.1 Commands to add

Add these commands:

```text
imu status
imu once
imu stream on
imu stream off
imu calibrate gyro
```

Add these future magnetometer placeholders:

```text
mag status
mag once
mag calibrate
```

The `mag` commands should return `not implemented` for now.

### 8.2 `terminal_parser.h`

Add enum values:

```c
TCMD_IMU_STATUS,
TCMD_IMU_ONCE,
TCMD_IMU_STREAM_ON,
TCMD_IMU_STREAM_OFF,
TCMD_IMU_CALIBRATE_GYRO,

TCMD_MAG_STATUS,
TCMD_MAG_ONCE,
TCMD_MAG_CALIBRATE,
```

Do not reorder existing enum values unless unavoidable. Append new values after existing command types to reduce regression risk.

### 8.3 `terminal_parser.c`

Parse exact lowercase commands:

```text
imu status
imu once
imu stream on
imu stream off
imu calibrate gyro

mag status
mag once
mag calibrate
```

Follow the existing parser style. Keep command matching deterministic and simple.

### 8.4 `command_handler.c`

Add cases:

```c
case TCMD_IMU_STATUS:
    ImuManager_PrintStatus();
    break;

case TCMD_IMU_ONCE:
    ImuManager_PrintOnce();
    break;

case TCMD_IMU_STREAM_ON:
    ImuManager_SetStreamEnabled(true);
    break;

case TCMD_IMU_STREAM_OFF:
    ImuManager_SetStreamEnabled(false);
    break;

case TCMD_IMU_CALIBRATE_GYRO:
    ImuManager_CalibrateGyro();
    break;
```

Future magnetometer placeholder cases:

```c
case TCMD_MAG_STATUS:
case TCMD_MAG_ONCE:
case TCMD_MAG_CALIBRATE:
    Logger_Log(LOG_INFO, "[MAG] not implemented");
    break;
```

### 8.5 DISARM policy

IMU commands are diagnostics/telemetry commands, not motion commands. They may be allowed while DISARM is active.

Do not accidentally block these commands through the DISARM safety filter unless the project policy explicitly requires it.

### 8.6 Help text

Update `CommandHandler_PrintHelp()` with:

```text
IMU commands:
  imu status          Show IMU detection/health/stream state
  imu once            Print one IMU telemetry sample
  imu stream on       Enable periodic IMU telemetry, max 10 Hz
  imu stream off      Disable periodic IMU telemetry
  imu calibrate gyro  Calibrate gyro offset while sensor is stationary

Future magnetometer placeholders:
  mag status          Show future magnetometer placeholder status
  mag once            Not implemented yet
  mag calibrate       Not implemented yet
```

---

## 9. Stage 5 — Logger and Telemetry Format

### 9.1 Status logs

Status logs may use the existing logger:

```text
[INFO] [IMU] status: present=1 initialized=1 healthy=1 stream=0 who=0x71 fail=0
[INFO] [IMU] stream ON
[INFO] [IMU] stream OFF
[INFO] [MAG] not implemented
```

### 9.2 Telemetry lines

Telemetry lines should be easy for `earendil.py` to parse.

Preferred raw telemetry format:

```text
[IMU] AX:<float> AY:<float> AZ:<float> GX:<float> GY:<float> GZ:<float>
```

Example:

```text
[IMU] AX:0.01 AY:-0.02 AZ:1.00 GX:0.12 GY:-0.04 GZ:0.08
```

When a real magnetometer is added in the future, extend the same line format:

```text
[IMU] AX:<float> AY:<float> AZ:<float> GX:<float> GY:<float> GZ:<float> MX:<float> MY:<float> MZ:<float>
```

Do not output fake `MX`, `MY`, or `MZ` values in the first implementation.

### 9.3 Raw-line helper

If the existing logger always prepends `[INFO]`, add a raw terminal helper instead of forcing IMU telemetry through normal logging.

Possible API:

```c
void Logger_WriteRaw(const char *text);
```

or:

```c
void Logger_WriteRawLine(const char *line);
```

Telemetry output must end with `\r\n`.

### 9.4 Avoid console spam

Default boot state:

```text
IMU stream OFF
```

Only these operations print telemetry:

```text
imu once
imu stream on
```

`imu stream off` must stop periodic telemetry completely.

---

## 10. Stage 6 — `earendil.py` GUI Integration

### 10.1 Preserve current GUI behavior

Do not break these GUI features:

```text
Serial connect/disconnect
H7 Console
GUI Console
Motor State table
F411 telemetry parsing
F411 tuning Settings dialog
Operating Mode DISARM/MANUAL/AUTONOMOUS buttons
RPM/DUTY mode switching
W/A/S/D driving
Shift/Ctrl FB value adjustment
Num+/Num- ROT value adjustment
Theme switching
```

### 10.2 Keep existing IMU/Mag placeholders

`earendil.py` already has an IMU placeholder table with:

```text
Accel X/Y/Z
Gyro X/Y/Z
Mag X/Y/Z
```

Keep the Mag row as a placeholder for future magnetometer support. In the first firmware stage, only Accel and Gyro values will update. Mag values must remain `--` until a real magnetometer driver exists.

### 10.3 Add IMU parser

Add a parser near the other regex definitions:

```python
_RE_IMU_DATA = re.compile(
    r"(?:^\[INFO\]\s*)?\[IMU\]\s+"
    r"AX:\s*([-\d.]+)\s+AY:\s*([-\d.]+)\s+AZ:\s*([-\d.]+)\s+"
    r"GX:\s*([-\d.]+)\s+GY:\s*([-\d.]+)\s+GZ:\s*([-\d.]+)"
    r"(?:\s+MX:\s*([-\d.]+)\s+MY:\s*([-\d.]+)\s+MZ:\s*([-\d.]+))?"
)
```

### 10.4 Implement `_parse_imu_line()`

Add:

```python
def _parse_imu_line(self, line: str) -> bool:
    match = _RE_IMU_DATA.match(line)
    if not match:
        return False

    try:
        values = {
            "AX": float(match.group(1)),
            "AY": float(match.group(2)),
            "AZ": float(match.group(3)),
            "GX": float(match.group(4)),
            "GY": float(match.group(5)),
            "GZ": float(match.group(6)),
        }

        if match.group(7) is not None:
            values["MX"] = float(match.group(7))
            values["MY"] = float(match.group(8))
            values["MZ"] = float(match.group(9))

        self._update_imu_values(values)
        return True
    except ValueError:
        return False
```

### 10.5 Prevent IMU stream from flooding the H7 Console

Change `_on_rx_line()` so IMU telemetry is parsed before normal console logging.

Target behavior:

```python
def _on_rx_line(self, line: str):
    if self._parse_imu_line(line):
        return

    self._log_rx(line)
    self._parse_motor_telemetry_line(line)
    self._parse_rx_for_motor_state(line)
    self._parse_uart_error_line(line)
    self._parse_operating_mode_confirm(line)
```

If the existing GUI code has slightly different parser order, preserve all existing parser calls but make sure IMU telemetry lines return early before `_log_rx(line)`.

### 10.6 Add IMU controls

Add a compact control row inside the IMU group:

```text
IMU Status
IMU Once
Stream ON/OFF
Calibrate Gyro
```

Button commands:

```text
IMU Status       -> imu status
IMU Once         -> imu once
Stream ON        -> imu stream on
Stream OFF       -> imu stream off
Calibrate Gyro   -> imu calibrate gyro
```

The stream button can be a toggle button. When checked, send `imu stream on`; when unchecked, send `imu stream off`.

### 10.7 Future Mag controls

Do not add active magnetometer controls yet unless they are clearly disabled.

Optional disabled placeholder:

```text
Mag: Future
```

Do not send `mag` commands automatically.

---

## 11. Stage 7 — Manual Test Plan

### 11.1 Baseline regression test before IMU

Before testing IMU, verify the existing rover commands:

```text
help
status
mode
mode disarm
mode manual
mode auto
m speed
m duty
stop
brake
termstat
```

Expected: all existing commands work as before.

### 11.2 IMU disconnected test

Run firmware with the IMU physically disconnected.

Expected:

```text
Firmware boots.
help works.
status works.
mode disarm/manual/auto work.
Motor commands still work according to existing safety policy.
imu status reports not present / not initialized.
imu once reports not ready without blocking.
No Error_Handler.
No lockup.
No continuous IMU spam.
```

### 11.3 WHO_AM_I test

Connect the IMU and run:

```text
imu status
```

Expected example:

```text
[INFO] [IMU] status: present=1 initialized=1 healthy=1 stream=0 who=0x71 fail=0
```

Any of these WHO_AM_I values may be accepted:

```text
0x71
0x73
0x70
0x68
```

If the value is `0x00` or `0xFF`, check wiring, address, pull-ups, power, and NCS/CSB/SA0 pins.

### 11.4 Single telemetry test

Run:

```text
imu once
```

Expected one telemetry line only:

```text
[IMU] AX:0.01 AY:-0.02 AZ:1.00 GX:0.12 GY:-0.04 GZ:0.08
```

`earendil.py` should update Accel and Gyro rows. Mag row should remain `--`.

### 11.5 Stream test

Run:

```text
imu stream on
```

Expected:

```text
IMU telemetry emitted at max 10 Hz.
earendil.py IMU table updates.
H7 Console does not fill with IMU telemetry lines because GUI parses them early.
```

Then run:

```text
imu stream off
```

Expected:

```text
Periodic IMU telemetry stops.
```

### 11.6 Operating mode regression while IMU stream is ON

With stream enabled, test:

```text
mode disarm
mode manual
mode auto
```

Expected H7 output still contains:

```text
[MODE] DISARM active
[MODE] MANUAL active
[MODE] AUTONOMOUS active
```

`earendil.py` operating-mode indicator must update only after receiving those confirmation lines.

### 11.7 Motor/telemetry regression while IMU stream is ON

With IMU stream enabled, verify:

```text
F411 motor telemetry still updates.
Motor State table still updates.
UART error parsing still updates the Error column.
F411 tuning Settings dialog still sends paced commands correctly.
W/A/S/D driving still sends motion commands.
Stop and brake still work.
```

---

## 12. Stage 8 — Future Magnetometer Plan

The current IMU module must not pretend to have magnetometer data.

Future magnetometer should be added as a separate driver and manager layer, for example:

```text
Core/Inc/mag_driver_<sensor>.h
Core/Src/mag_driver_<sensor>.c
Core/Inc/mag_manager.h
Core/Src/mag_manager.c
```

Possible future sensors:

```text
QMC5883L
HMC5883L
LIS3MDL
AK8963, only if a real accessible AK8963 exists
```

Future telemetry may extend the same `[IMU]` line:

```text
[IMU] AX:... AY:... AZ:... GX:... GY:... GZ:... MX:... MY:... MZ:...
```

Only after real magnetometer values are implemented should `MX`, `MY`, and `MZ` be emitted.

Do not add fake heading, compass, yaw correction, hard-iron calibration, or soft-iron calibration in the first 6-axis IMU stage.

---

## 13. Final Acceptance Criteria

The implementation is acceptable only if all of these are true:

```text
1. Firmware boots with IMU disconnected.
2. Existing terminal commands still work.
3. Existing operating mode confirmation lines are unchanged.
4. Existing motor UART, safety, telemetry, and tuning systems are not regressed.
5. I2C1 PB8/PB9 IMU connection works.
6. imu status reports a real WHO_AM_I value when IMU is connected.
7. imu once prints exactly one telemetry line.
8. imu stream on emits telemetry at max 10 Hz.
9. imu stream off stops telemetry.
10. No IMU telemetry is printed continuously by default after boot.
11. earendil.py updates Accel and Gyro rows from IMU telemetry.
12. earendil.py keeps Mag row as a future placeholder and leaves it as -- until MX/MY/MZ exist.
13. earendil.py does not flood H7 Console with IMU stream lines.
14. IMU failure never calls Error_Handler and never blocks App_Update().
15. Future magnetometer support remains planned but not faked.
```

---

## 14. Implementation Order Summary

Follow this exact order:

```text
1. Confirm the current rover firmware works without IMU.
2. Add I2C1 infrastructure: hi2c1, MX_I2C1_Init, PB8/PB9 MSP.
3. Add imu_mpu6axis.h/c.
4. Implement WHO_AM_I read.
5. Implement MPU wake-up and basic accel/gyro config.
6. Implement 14-byte burst read from 0x3B.
7. Convert raw accel/gyro to g and deg/s.
8. Add imu_manager.h/c.
9. Add non-blocking retry/update/stream state machine.
10. Add imu terminal commands to parser and command handler.
11. Add telemetry raw output format.
12. Add earendil.py IMU parser and IMU buttons.
13. Prevent IMU telemetry from flooding H7 Console.
14. Test IMU disconnected case.
15. Test WHO_AM_I.
16. Test imu once.
17. Test imu stream on/off.
18. Test mode disarm/manual/auto while IMU stream is ON.
19. Test motor telemetry and motion commands while IMU stream is ON.
20. Leave magnetometer placeholders intact for future real magnetometer support.
```

