# Earendil Main Firmware

STM32H723ZG rover main-controller firmware for a four-motor skid-steer rover platform. This firmware is the central command, safety, telemetry, and configuration bridge between a PC/GUI terminal and four independent STM32F411-based BLDC motor-controller boards.

The H7 firmware provides:

- USART3 terminal / GUI command interface
- Four dedicated motor UART links: `FL`, `FR`, `RL`, `RR`
- DMA-based TX/RX handling for motor UARTs
- RPM and PWM/duty control modes
- DISARM / MANUAL / AUTONOMOUS rover operating modes
- Direct per-motor raw command forwarding
- Motor tuning command forwarding
- Motor configuration readback cache through `cfgread` / `cfgcache`
- Link-loss detection and UART diagnostics
- MPU9250 IMU commands over I2C1
- QMC5883P magnetometer commands over I2C1
- GUI-friendly telemetry lines for motor, IMU, and magnetometer data

---

## 1. Hardware Target

| Item | Value |
|---|---|
| Main MCU | STM32H723ZG / STM32H723ZGTX class MCU |
| Firmware framework | STM32 HAL / STM32CubeMX-style project |
| Main terminal UART | USART3 |
| Motor UART count | 4 independent UART links |
| Motor-controller side | STM32F411CEU6 BLDC motor driver boards |
| Motor topology | 4 Hall-sensored BLDC hub motors |
| Rover drive type | Four-wheel skid-steer / differential drive |
| IMU bus | I2C1 |
| IMU target | MPU9250 / MPU6500-compatible accel/gyro block |
| Magnetometer target | QMC5883P |

The H7 does **not** directly perform BLDC commutation. It sends command strings to the F411 motor-controller boards. The F411 boards handle motor-level commutation, Hall feedback, PWM generation, and low-level motor control.

### Raspberry Pi TCP bridge deployment

For remote control, the deployed path is:

```text
PC GUI -> network -> Raspberry Pi tcp_uart_bridge.py -> serial -> H7
```

Run serial-device discovery on the Raspberry Pi, because that is where the H7
is physically attached:

```bash
ls -l /dev/serial/by-id/
```

Start the bridge on the Pi with its LAN interface exposed and restrict access
to the control PC (or another appropriate private CIDR):

```bash
python3 tcp_uart_bridge.py \
  --host 0.0.0.0 --port 5000 \
  --serial-device /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_XXXX-if02 \
  --baud 115200 \
  --allow-client 192.168.50.10/32 \
  --log-file /tmp/bridge.log
```

The GUI connects to the Raspberry Pi LAN address, for example
`192.168.50.20:5000`. Use `127.0.0.1` only when the GUI and bridge are
intentionally running on the same machine. The bridge logs to stderr by
default; `/tmp/bridge.log` exists only when `--log-file /tmp/bridge.log` is
used. Without that option, capture both output streams explicitly:

```bash
python3 tcp_uart_bridge.py [options] 2>&1 | tee /tmp/bridge.log
```

---

## 2. Repository Layout

```text
.
├── Core/
│   ├── Inc/                         # Application headers and HAL headers
│   └── Src/                         # Application source files and CubeMX-generated code
├── Drivers/                         # STM32 HAL/CMSIS driver tree
├── Debug/                           # Debug/build output directory
├── H7-DMA.ioc                       # STM32CubeMX configuration file
├── earendil.py                      # Optional PySide6 desktop GUI / serial tool
├── earendil_logo.png                # GUI/logo asset
├── DMA_TX_Roadmap.md                # TX DMA design notes
├── H723_HAL_UART_DMA_Roadmap.md     # UART/DMA roadmap notes
├── dma-tx-report.md                 # DMA TX implementation report
├── STM32H723ZGTX_FLASH.ld           # Flash linker script
└── STM32H723ZGTX_RAM.ld             # RAM linker script
```

Important firmware modules:

| File | Responsibility |
|---|---|
| `Core/Src/main.c` | HAL startup, clock config, peripheral init, main loop handoff |
| `Core/Src/app_main.c` | Application initialization and periodic task loop |
| `Core/Src/terminal_if.c` | USART3 terminal RX/TX interface |
| `Core/Src/terminal_parser.c` | Text command parser |
| `Core/Src/command_handler.c` | Command execution, safety gate, mode switching, IMU/mag/tuning dispatch |
| `Core/Src/motion_controller.c` | High-level motion to per-wheel command conversion |
| `Core/Src/motor_dispatcher.c` | Broadcast and per-motor UART dispatch |
| `Core/Src/motor_protocol.c` | Motor command string encoding |
| `Core/Src/motor_uart_dma.c` | Motor UART RX DMA, line extraction, telemetry/error handling |
| `Core/Src/motor_tx_dma.c` | Motor UART TX DMA, busy/pending policy, stop/brake priority |
| `Core/Src/motor_tuning_config.c` | Per-motor `cfg` response parser and tuning cache |
| `Core/Src/safety_manager.c` | DISARM behavior, stop/brake safety actions, link-loss handling |
| `Core/Src/operating_mode.c` | Rover operating mode owner |
| `Core/Src/control_mode.c` | RPM/PWM control mode owner |
| `Core/Src/activity_light.c` | Rover mode LED outputs |
| `Core/Src/i2c_scanner.c` | I2C bus scan command |
| `Core/Src/imu_mpu9250.c` | MPU9250/MPU6500 accel/gyro readout and filtering |
| `Core/Src/mag_qmc5883p.c` | QMC5883P magnetometer readout |
| `Core/Inc/app_config.h` | UART mapping, buffer sizes, timeouts |
| `Core/Inc/rover_types.h` | Shared motor, direction, link, and rover-mode types |

---

## 3. System Architecture

```text
              PC / GUI / Serial Terminal
                    115200 8N1
                         │
                         │ USART3
                         ▼
┌────────────────────────────────────────────────────┐
│              STM32H723ZG Main Controller            │
├────────────────────────────────────────────────────┤
│ Terminal interface                                  │
│ Command parser / command handler                    │
│ DISARM / MANUAL / AUTONOMOUS safety gate            │
│ RPM/PWM control mode manager                        │
│ Motion controller                                   │
│ Motor UART dispatcher                               │
│ Motor TX/RX DMA                                     │
│ Motor tuning config cache                           │
│ I2C IMU + magnetometer diagnostics                  │
└───────┬────────────┬────────────┬────────────┬───────┘
        │            │            │            │
      USART2        UART4        UART7        UART5
       FL            FR           RL           RR
        │            │            │            │
        ▼            ▼            ▼            ▼
   F411 BLDC    F411 BLDC    F411 BLDC    F411 BLDC
   Front Left   Front Right  Rear Left    Rear Right
```

The PC/GUI sends high-level text commands to the H7. The H7 validates the command, applies the current safety/mode policy, converts motion commands into motor-specific commands, and transmits them to the proper F411 controller.

---

## 4. Detailed Pinout

### 4.1 Terminal / Host Interface

| Function | Peripheral | MCU Pin | Direction from H7 | Notes |
|---|---:|---:|---|---|
| Terminal TX | USART3_TX | PD8 | H7 → PC/host | Logs and command responses |
| Terminal RX | USART3_RX | PD9 | PC/host → H7 | Operator / GUI command input |

Serial settings:

```text
Baud rate : 115200
Data bits : 8
Parity    : None
Stop bits : 1
Flow ctrl : None
Mode      : TX/RX
```

USART3 is the main terminal interface. The firmware prints command responses, telemetry forwarding, warnings, and error messages through this port.

---

### 4.2 Motor UART Pinout

| Motor | Position | Peripheral | H7 TX Pin | H7 RX Pin | Connection Rule | DMA |
|---|---|---:|---:|---:|---|---|
| `FL` | Front Left | USART2 | PD5 | PD6 | H7 TX → F411 RX, F411 TX → H7 RX | RX/TX DMA |
| `FR` | Front Right | UART4 | PD1 | PD0 | H7 TX → F411 RX, F411 TX → H7 RX | RX/TX DMA |
| `RL` | Rear Left | UART7 | PE8 | PE7 | H7 TX → F411 RX, F411 TX → H7 RX | RX/TX DMA |
| `RR` | Rear Right | UART5 | PC12 | PD2 | H7 TX → F411 RX, F411 TX → H7 RX | RX/TX DMA |

All motor UARTs use:

```text
Baud rate : 115200
Data bits : 8
Parity    : None
Stop bits : 1
Flow ctrl : None
Mode      : TX/RX
Signal    : 3.3 V TTL UART
```

Wiring rule:

```text
H7 TX  -> F411 RX
H7 RX  <- F411 TX
GND    <-> GND
```

Do not connect these pins directly to RS-232 voltage levels.

---

### 4.3 I2C1 Sensor Pinout

| Function | Peripheral | MCU Pin | Notes |
|---|---:|---:|---|
| I2C1 SCL | I2C1_SCL | PB8 | Open-drain I2C clock |
| I2C1 SDA | I2C1_SDA | PB9 | Open-drain I2C data |

Expected devices:

| Device | Typical 7-bit Address | Related Commands |
|---|---:|---|
| MPU9250 / MPU6500 accel-gyro | `0x68` or `0x69` | `mpuwho`, `mpuinit`, `mpuraw`, `mpuconv`, etc. |
| QMC5883P magnetometer | `0x2C` typical in this firmware context | `magwho`, `maginit`, `magraw`, `magimu` |

Use proper I2C pull-up resistors on SCL/SDA if the sensor module does not already include them.

---

### 4.4 Rover Mode LED Pinout

| LED Meaning | MCU Pin | Rover Mode |
|---|---:|---|
| Red LED | PB0 | DISARM |
| Green LED | PB1 | MANUAL |
| Yellow LED | PB2 | AUTONOMOUS |

At boot, the firmware enters DISARM mode. In DISARM, motion commands are locked and the red LED is active.

---

### 4.5 Motor UART DMA Mapping

| Peripheral | Function | DMA Stream | Request | Motor |
|---|---|---:|---|---|
| UART4 | RX | DMA1 Stream0 | UART4_RX | FR |
| UART5 | RX | DMA1 Stream1 | UART5_RX | RR |
| UART7 | RX | DMA1 Stream2 | UART7_RX | RL |
| USART2 | RX | DMA1 Stream3 | USART2_RX | FL |
| UART4 | TX | DMA1 Stream4 | UART4_TX | FR |
| UART5 | TX | DMA1 Stream5 | UART5_TX | RR |
| UART7 | TX | DMA1 Stream6 | UART7_TX | RL |
| USART2 | TX | DMA1 Stream7 | USART2_TX | FL |

The motor RX path uses receive-to-idle DMA behavior. The TX path uses DMA with a busy/pending policy and special priority handling for safety commands such as `stop` and `x`.

---

## 5. Operating Model

The firmware has two independent mode layers.

### 5.1 Rover Operating Mode

| Mode | Motion Allowed? | Description |
|---|---:|---|
| `DISARM` | No | Default boot/safety mode. Motion-causing commands are blocked. |
| `MANUAL` | Yes | Operator/GUI commands can move the rover. |
| `AUTONOMOUS` | Yes | Autonomous state flag for higher-level control. |

DISARM is a logical safety lock, not a low-power MCU state. The H7 main loop, terminal, sensors, and diagnostics remain active.

### 5.2 Motor Control Mode

| Control Mode | Command | Meaning |
|---|---|---|
| RPM / speed | `m speed` or `mode speed` | Motion commands are encoded as RPM commands |
| PWM / duty | `m duty` or `mode duty` | Motion commands are encoded as duty/PWM commands |

When changing RPM/PWM mode, the firmware first sends `stop`, waits for motor TX DMA to drain, then sends the requested motor mode command. This avoids desynchronizing the H7 local mode from the F411 motor-controller mode.

---

## 6. Command Reference

### 6.1 General Commands

| Command | Description |
|---|---|
| `help` | Print command list |
| `status` | Send `status` to all motor controllers |
| `identify` | Send service arm command, wait for TX drain, then send `identify` to all motor controllers |
| `stop` | Stop all motors |
| `brake` | Send brake command `x` to all motor controllers |
| `termstat` | Print terminal RX queue diagnostics |
| `i2cscan` | Scan the I2C1 bus |

---

### 6.2 Rover Operating Mode Commands

| Command | Description |
|---|---|
| `mode` | Print current rover operating mode |
| `mode disarm` | Enter DISARM mode, stop/brake motors, lock motion |
| `mode manual` | Enter MANUAL mode, motors remain stopped until a fresh motion command arrives |
| `mode auto` | Enter AUTONOMOUS mode |
| `mode autonomous` | Alias for `mode auto` |

---

### 6.3 RPM Mode Motion Commands

These commands are valid when the H7 control mode is RPM/speed mode.

| Command | Range | Description |
|---|---:|---|
| `f0` ... `f200` | 0–200 RPM | Forward |
| `b0` ... `b200` | 0–200 RPM | Backward |
| `r0` ... `r200` | 0–200 RPM | Turn right |
| `l0` ... `l200` | 0–200 RPM | Turn left |

Examples:

```text
m speed
mode manual
f100
r60
stop
```

---

### 6.4 PWM / Duty Mode Motion Commands

These commands are valid when the H7 control mode is PWM/duty mode.

| Command | Range | Description |
|---|---:|---|
| `fd0` ... `fd4000` | 0–4000 | Forward duty/PWM command |
| `bd0` ... `bd4000` | 0–4000 | Backward duty/PWM command |
| `rd0` ... `rd4000` | 0–4000 | Right-turn duty/PWM command |
| `ld0` ... `ld4000` | 0–4000 | Left-turn duty/PWM command |

Examples:

```text
m duty
mode manual
fd1200
rd800
stop
```

---

### 6.5 Arc-Turn Drive Commands

Arc-turn commands allow outer/inner wheel scaling with a turn ratio.

```text
drive rpm  <0..200>  <fl|fr|bl|br>  tr <0.00..1.00>
drive duty <0..4000> <fl|fr|bl|br>  tr <0.00..1.00>
```

Examples:

```text
drive rpm 100 fl tr 0.50
drive rpm 100 fr tr 0.50
drive duty 2000 bl tr 0.50
drive duty 2000 br tr 0.50
```

---

### 6.6 Direct Per-Motor Raw Commands

Direct commands forward raw text to one selected motor controller only.

```text
FL <raw text>
FR <raw text>
RL <raw text>
RR <raw text>
```

Examples:

```text
FL status
FR identify
RL cfg
RR mode speed
```

In DISARM, only safe raw payloads are allowed, such as:

```text
status
identify
stop
x
cfg
mode speed
mode duty
```

Motion-causing direct raw commands such as `FL f100` are blocked while DISARM is active.

---

## 7. Motor Tuning Commands

The H7 can validate and forward tuning commands to one motor or all motors.

### 7.1 Single-Motor Tuning

Use one of the motor tags: `FL`, `FR`, `RL`, `RR`.

| Command | Description |
|---|---|
| `FL base P1 P2 P3 P4 P5 P6 P7 P8` | Set 8 base PWM table values for FL |
| `FL boost P1 P2 P3 P4 P5 P6 P7 P8 MS` | Set 8 boost PWM table values and boost duration |
| `FL kickduty VALUE` | Set kick duty |
| `FL kick duty VALUE` | Alternate kick duty syntax |
| `FL kickms VALUE` | Set kick duration in ms |
| `FL kick ms VALUE` | Alternate kick ms syntax |
| `FL ramp UP DOWN` | Set ramp-up and ramp-down values |
| `FL pi KP KI` | Set PI gains |
| `FL telper MS` | Set motor telemetry period |

Examples:

```text
FL base 300 500 800 1100 1400 1700 2000 2300
FL boost 600 800 1000 1200 1400 1600 1800 2000 250
FL kickduty 900
FL kickms 50
FL ramp 20 30
FL pi 10.000 10.000
FL telper 100
```

### 7.2 Broadcast Tuning

The same tuning operations can be broadcast to all motors with `ALL`:

```text
ALL base 300 500 800 1100 1400 1700 2000 2300
ALL boost 600 800 1000 1200 1400 1600 1800 2000 250
ALL kickduty 900
ALL kickms 50
ALL ramp 20 30
ALL pi 10.000 10.000
ALL telper 100
```

---

## 8. Config Readback and Cache

The firmware includes a per-motor tuning configuration cache. The cache is populated by asking a motor controller to print its current config using the raw `cfg` command, then parsing the returned F411 lines.

### 8.1 Config Read Commands

| Command | Description |
|---|---|
| `cfgread FL` | Send `cfg` to the FL motor controller and parse/cache the response |
| `cfgread FR` | Send `cfg` to the FR motor controller and parse/cache the response |
| `cfgread RL` | Send `cfg` to the RL motor controller and parse/cache the response |
| `cfgread RR` | Send `cfg` to the RR motor controller and parse/cache the response |
| `cfgread all` | Send `cfg` to all four motor controllers |

Examples:

```text
cfgread RL
cfgread all
```

Internally, `cfgread RL` is equivalent to forwarding:

```text
RL cfg
```

### 8.2 Config Cache Print Commands

| Command | Description |
|---|---|
| `cfgcache` | Print cached tuning config for all motors |
| `cfgcache FL` | Print cached config for FL |
| `cfgcache FR` | Print cached config for FR |
| `cfgcache RL` | Print cached config for RL |
| `cfgcache RR` | Print cached config for RR |

Example:

```text
cfgcache RL
```

Typical cached output format:

```text
[CFG][RL] valid=1 updates=1 age_ms=1234
[CFG][RL] Kp_m=10000 Ki_m=10000 Kp=10.000 Ki=10.000
[CFG][RL] Base 300 500 800 1100 1400 1700 2000 2300
[CFG][RL] Boost 600 800 1000 1200 1400 1600 1800 2000 ms=250
[CFG][RL] Ramp up=20 down=30
[CFG][RL] Kick ON duty=900 ms=50
[CFG][RL] TelPer=100
```

### 8.3 Parsed F411 Config Lines

The H7 parser recognizes these F411 config response patterns:

| F411 Line Pattern | Cached Fields |
|---|---|
| `Kp_m=<int> Ki_m=<int>` | PI gains as fixed-point milliscale integers |
| `Base <P1> ... <P8>` | 8 base PWM table values |
| `Boost <P1> ... <P8> ms=<MS>` | 8 boost PWM table values and boost duration |
| `Ramp up=<UP> down=<DOWN>` | Ramp-up and ramp-down values |
| `Kick ON duty=<DUTY> ms=<MS>` | Kick enabled, duty, and duration |
| `Kick OFF duty=<DUTY> ms=<MS>` | Kick disabled, duty, and duration |
| `TelPer=<MS>` | Motor telemetry period |

The cache becomes valid only after the required PI, Base, and Boost parts are received. Optional fields such as Ramp, Kick, and TelPer can update the cache before or after the required fields.

If a motor replies with `[ERR] Unknown command` after `cfg`, the firmware marks that motor config read as unsupported and stores `unsupported cfg` in the cache error field.

---

## 9. IMU Commands

The firmware includes MPU9250/MPU6500-oriented I2C diagnostic and telemetry commands.

| Command | Description |
|---|---|
| `mpuwho` | Read WHO_AM_I register |
| `mpuregs` | Read diagnostic registers |
| `mpuwarm` | Probe before/after I2C warm-up |
| `mpuinit` | Basic accel/gyro initialization |
| `mpucfgtest` | CONFIG register write/readback diagnostic |
| `mpuraw` | One-shot raw accel/gyro/temperature read |
| `mpudbgraw` | Update IMU raw debug variables for CubeIDE |
| `mpugyrotest` | Diagnose gyro raw registers and gyro enable state |
| `mpuconv` | Read converted accel/gyro/temp values |
| `mpubias` | Print gyro static bias state |
| `mpubiason` | Enable gyro bias correction |
| `mpubiasoff` | Disable gyro bias correction |
| `mpubiasclear` | Clear gyro bias to zero |

GUI-friendly converted output includes an `MPU_IMU` line containing accel, gyro, temperature, bias/filter status, and OK flag.

---

## 10. IMU Stream / Filter Commands

| Command | Description |
|---|---|
| `imu help` | Show IMU command list |
| `imu stream on` | Enable periodic IMU telemetry |
| `imu stream off` | Disable periodic IMU telemetry |
| `imu telper <MS>` | Set IMU telemetry period, typically 20–5000 ms |
| `imu gyrofilter status` | Print gyro output filter settings |
| `imu gyrofilter on` | Enable gyro output filter |
| `imu gyrofilter off` | Disable gyro output filter |
| `imu deadband <VALUE>` | Set gyro display deadband |
| `imu lpf <ALPHA>` | Set gyro exponential moving average alpha |

Example:

```text
imu stream on
imu telper 50
imu gyrofilter on
imu deadband 20
imu lpf 100
```

---

## 11. Magnetometer Commands

| Command | Description |
|---|---|
| `magwho` | Detect QMC5883P magnetometer |
| `maginit` | Initialize QMC5883P |
| `magraw` | Read raw magnetometer X/Y/Z |
| `magimu` | Read compact GUI-friendly magnetometer X/Y/Z |
| `maghelp` | Show magnetometer command list |

Example:

```text
magwho
maginit
magraw
magimu
```

---

## 12. Motor Telemetry Format

The F411 motor controllers can send compact telemetry lines similar to:

```text
RPM:60,T:0,D:0,DIR:N,APP_PH:2,SP:1,BRAKE:1,FC:0,H:4,PWM_SET:0,PWM_ACT:0,QDROP:0,RXB:23695
```

Common fields:

| Field | Meaning |
|---|---|
| `RPM` | Measured motor speed |
| `T` | Target value or target-related field, depending on F411 firmware |
| `D` | Duty/control-related value, depending on F411 firmware |
| `DIR` | Direction state |
| `APP_PH` | Applied phase / commutation phase information |
| `SP` | Speed/control state flag |
| `BRAKE` | Brake state |
| `FC` | Fault code |
| `H` | Hall sensor state |
| `PWM_SET` | Requested PWM value |
| `PWM_ACT` | Applied PWM value |
| `QDROP` | Dropped command / queue drop counter |
| `RXB` | Received byte counter |

The H7 telemetry classifier treats payloads starting with `RPM:` and containing motor telemetry fields such as `PWM_ACT:` and `RXB:` as telemetry, not config-cache data.

---

## 13. DISARM Safety Policy

While the rover is in DISARM:

Allowed command groups include:

- `help`
- `status`
- `termstat`
- `i2cscan`
- IMU and magnetometer diagnostic/config commands
- `cfgread` and `cfgcache`
- rover mode transitions
- safe direct motor raw commands: `status`, `identify`, `stop`, `x`, `cfg`, `mode speed`, `mode duty`

Blocked command groups include:

- normal motion commands such as `f100`, `b100`, `r100`, `l100`
- PWM motion commands such as `fd1000`, `bd1000`, `rd1000`, `ld1000`
- arc-drive motion commands
- unsafe direct raw motor commands such as `FL f100`

This prevents accidental motion while still allowing diagnostics, configuration readback, stop/brake, and sensor checks.

---

## 14. Typical Bring-Up Sequence

```text
help
mode
status
i2cscan
mpuwho
magwho
cfgread all
cfgcache
mode manual
m speed
f50
stop
mode disarm
```

For PWM/duty testing:

```text
mode manual
m duty
fd500
stop
mode disarm
```

For one motor only:

```text
mode manual
FL status
FL cfg
cfgcache FL
FL mode speed
FL stop
```

---

## 15. Notes for GUI Integration

The included `earendil.py` GUI can use the terminal port to:

- send rover mode commands
- send RPM/PWM motion commands
- send per-motor raw commands
- display motor telemetry table values
- display UART/link error state
- request motor tuning config with `cfgread`
- show cached tuning config using `cfgcache`
- display IMU converted telemetry from `MPU_IMU`
- display magnetometer telemetry from `MAG_IMU`

Recommended GUI flow for motor tuning panels:

```text
1. User selects motor, e.g. RL
2. GUI sends: cfgread RL
3. GUI waits briefly for F411 cfg lines to be parsed by H7
4. GUI sends: cfgcache RL
5. GUI parses the H7 [CFG][RL] cached output and updates the UI fields
```

---

## 16. Build / Flash Notes

This is an STM32CubeIDE / STM32 HAL project. Typical workflow:

1. Open the project in STM32CubeIDE.
2. Verify the selected target is STM32H723ZG / STM32H723ZGTX.
3. Build the firmware.
4. Flash through ST-LINK.
5. Open the USART3 virtual COM port at `115200 8N1`.
6. Send `help` to verify the terminal interface.

---

## 17. Safety Notes

- Always test with current-limited supplies during bring-up.
- Keep the rover lifted or wheels unloaded during first motor tests.
- Confirm that DISARM blocks motion before connecting full motor power.
- Confirm each motor UART mapping with `FL status`, `FR status`, `RL status`, and `RR status`.
- Confirm direction mapping at low RPM/PWM before autonomous operation.
- Do not rely on software safety alone; add hardware emergency stop and power isolation in the full rover system.
