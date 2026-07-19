# AGENTS.md

## Scope

This file applies to the entire repository. A more specific `AGENTS.md` in a
subdirectory may override these rules for that subtree.

This is a safety-relevant rover-control project. Make the smallest correct
change, preserve existing hardware/protocol contracts, and do not broaden the
task without a concrete reason.

## Project Summary

The repository contains three deployed components:

1. **STM32H723 firmware** in `Core/Inc` and `Core/Src`.
2. **PC-side PySide6 GUI** in the root `earendil.py`.
3. **Raspberry Pi TCP-to-serial bridge** in the root
   `tcp_uart_bridge.py`, optionally launched by `earendil-bridge.service`.

The intended control path is:

```text
PC earendil.py --TCP--> Raspberry Pi tcp_uart_bridge.py --serial--> STM32H723
```

The STM32H723 validates operator commands, owns rover-level safety and mode
state, dispatches commands to four STM32F411 motor-controller boards, and
forwards telemetry. The F411 boards own BLDC commutation, Hall processing, PWM,
and low-level motor control.

## Source-of-Truth Files

- `README.md`: current documented behavior, pinout, commands, telemetry, and
  bring-up notes. Verify important details against code before changing them.
- `Core/Src/app_main.c`: application startup order and cooperative main loop.
- `Core/Src/terminal_parser.c` + `Core/Inc/terminal_parser.h`: command grammar
  and parsed command model.
- `Core/Src/command_handler.c`: command execution and safety gates.
- `Core/Src/operating_mode.c`: DISARM/MANUAL/AUTONOMOUS state owner.
- `Core/Src/control_mode.c`: RPM/PWM state owner.
- `Core/Src/motion_controller.c`: high-level motion and arc-turn calculation.
- `Core/Src/motor_dispatcher.c`: per-wheel dispatch.
- `Core/Src/motor_tx_dma.c`: TX queueing, DMA lifetime, and safety priority.
- `Core/Src/motor_uart_dma.c`: motor RX DMA, telemetry, link state, and UART
  error handling.
- `Core/Src/manipulation_uart_dma.c`: UART8 manipulation-controller link.
- `Core/Src/imu_mpu9250.c`, `Core/Src/mag_qmc5883p.c`, and
  `Core/Src/i2c_recovery.c`: shared I2C1 sensor path and recovery.
- Root `earendil.py`: production GUI. Files under `tools/` are auxiliary or
  historical variants; do not modify them unless the task explicitly names
  them.
- Root `tcp_uart_bridge.py`: production bridge.

Roadmaps and reports describe design history. If they conflict with current
code and `README.md`, do not treat them as the implementation source of truth.

## Files That Are Generated, Vendor-Owned, or Build Output

Do not edit these unless the task explicitly requires it:

- `Drivers/`: ST HAL/CMSIS vendor code.
- `Debug/`: generated build output and generated makefiles.
- `__pycache__/` and `tools/__pycache__/`: generated Python cache files.
- `*.elf`, `*.map`, `*.list`, `*.o`, `*.d`, `*.su`, and `*.cyclo`: generated
  artifacts.

`Core/Src/main.c`, `Core/Src/stm32h7xx_hal_msp.c`,
`Core/Src/stm32h7xx_it.c`, and related CubeMX files contain generated code.
Prefer application modules and `USER CODE BEGIN/END` regions. Changes outside
user regions must be narrowly justified and documented.

Do not edit `H7-DMA.ioc` unless the user explicitly requests a CubeMX/peripheral
configuration change. Do not regenerate CubeMX code as part of an unrelated
fix.

## Non-Negotiable Architecture Rules

### PC GUI

- `earendil.py` is a **TCP client**, not a serial terminal.
- Do not add serial-device, COM-port, baud-rate, or `pyserial` controls to the
  production GUI.
- The GUI must connect to the Raspberry Pi bridge using `QTcpSocket`.
- The H7 serial output is the source of truth for confirmed rover mode, link
  state, command results, and telemetry. Do not display a command as confirmed
  merely because the GUI transmitted it.
- Keep the GUI as one standalone root Python file unless the user explicitly
  requests a package or multi-file refactor.

### Raspberry Pi bridge

- `tcp_uart_bridge.py` is the only component that owns the H7 serial device in
  the deployed architecture.
- Preserve the required `--serial-device` argument and prefer persistent
  `/dev/serial/by-id/...` paths.
- Preserve single-active-client behavior, client allow-list validation,
  reconnect behavior, complete-write handling, and disconnect/shutdown safety
  stop behavior.
- Do not weaken access control, bind defaults, or safety-stop behavior merely
  to make connection testing easier.

### H7 firmware

- The H7 is the rover-level safety authority. The GUI and bridge must not bypass
  firmware validation.
- The H7 sends text commands to F411 boards; it does not implement BLDC
  commutation.
- Preserve the cooperative, continuously running `App_Update()` model.
- DISARM is a logical motion lock, not a low-power MCU state. Do not introduce
  `WFI`, STOP, STANDBY, or long sleeps into the normal application loop.

## Fixed Hardware Mapping

Do not reorder or reinterpret these mappings:

| Logical target | Peripheral | H7 TX | H7 RX |
|---|---|---|---|
| `FL` | USART2 | PD5 | PD6 |
| `FR` | UART4 | PD1 | PD0 |
| `RL` | UART7 | PE8 | PE7 |
| `RR` | UART5 | PC12 | PD2 |
| Terminal/host | USART3 | PD8 | PD9 |
| Manipulation F411 | UART8 | project CubeMX mapping | project CubeMX mapping |
| I2C1 SCL | I2C1 | PB8 | — |
| I2C1 SDA | I2C1 | PB9 | — |

All UART links currently use 115200 baud, 8 data bits, no parity, one stop bit,
and no flow control.

Expected I2C1 devices:

- MPU9250/MPU6500-compatible accel/gyro at `0x68` or `0x69`.
- QMC5883L magnetometer at `0x0D`.

`mag_qmc5883p.*` is a legacy filename; the implemented target is QMC5883L. Do
not rename this module casually because includes, documentation, and tooling
may depend on the existing path.

## Safety Invariants

Any change touching commands, motion, modes, UART TX, watchdogs, or networking
must preserve all of the following:

1. The rover boots in **DISARM**.
2. Motion-causing commands are rejected while DISARM is active.
3. Entering DISARM forces the existing stop/brake safety behavior and does not
   leave stale motion queued.
4. Switching RPM/PWM mode stops the motors and synchronizes the F411 mode before
   accepting commands in the new mode.
5. `stop`, `x`, and `brake` retain priority over queued normal motor commands.
6. The PC control-link heartbeat/watchdog remains active. Current intended
   timing is a 500 ms GUI heartbeat period and a 2000 ms H7 timeout.
7. Motor-link loss handling remains active. Current timeout is 3000 ms.
8. A reconnect or mode change must not replay stale motion. Fresh operator
   input is required after a safety transition.
9. Safety behavior must not depend solely on GUI state.
10. Never claim that a safety path works without either code-level evidence or
    an explicitly reported hardware test.

When uncertain, fail safe: stop motion, preserve diagnostics, and report the
uncertainty.

## Firmware Coding Rules

- Use GNU C11-compatible C and the existing STM32 HAL style.
- Keep declarations in headers and implementation in matching `.c` files.
- Prefer existing module APIs over direct access to another module's private
  state.
- Do not introduce heap allocation into firmware application code.
- Keep stack usage bounded; avoid large local arrays, especially in callbacks.
- Use fixed-size buffers and always reserve space for the terminating NUL when
  handling strings.
- Validate enum values, motor IDs, lengths, and numeric ranges at module
  boundaries.
- Current motion ranges are RPM `0..200` and duty/PWM `0..4000`.
- Prefer integer or fixed-point protocol values. Do not add text-to-float
  parsing to a real-time path when permille/fixed-point representation is
  sufficient.
- Avoid unrelated formatting, renaming, or broad refactors in bug-fix tasks.
- Preserve public function signatures unless every caller and header is updated
  in the same change.
- Keep comments focused on invariants, hardware constraints, protocol meaning,
  or non-obvious concurrency behavior.

## DMA, Interrupt, and Concurrency Rules

The STM32H7 memory/cache and callback model is critical.

- DMA buffers must remain alive and unchanged until transfer completion.
- Do not pass stack buffers or temporary command strings directly to DMA.
- Preserve the existing `.dma_buffer` placement and 32-byte alignment pattern
  for DMA-safe buffers.
- Do not remove cache/MPU handling without proving the replacement is correct
  for STM32H723 DMA visibility.
- Callback/ISR code must be short, bounded, and non-blocking.
- Do not call the blocking terminal logger from DMA/UART ISR completion paths.
- Defer logs and recovery work to periodic `Update()` functions using flags or
  counters.
- Shared state accessed by main-loop and callback context must remain `volatile`
  where appropriate and protected by the existing brief critical-section
  pattern.
- Preserve callback fan-out. For example, the shared UART RX event/error paths
  must continue routing UART8 to the manipulation module and motor UARTs to the
  motor module.
- When adding a UART, DMA stream, or callback consumer, update all relevant
  handle mappings, IRQ routing, callback dispatch, and diagnostics together.

## Main-Loop and Timing Rules

- `App_Update()` is a cooperative scheduler. New periodic work must return
  quickly.
- Do not add unbounded polling loops, long `HAL_Delay()` calls, or blocking I/O
  to normal periodic paths.
- Use `HAL_GetTick()` state machines for retries, telemetry periods, reconnects,
  and sensor recovery.
- A bounded synchronous wait is acceptable only where an existing safety
  transition requires it and the upper bound is explicit.
- Rate-limit repeated diagnostics. Avoid serial log floods that can starve
  command handling or change timing.

## Terminal Command and Protocol Changes

A command change is incomplete unless all affected layers remain consistent.
Check and update, as applicable:

1. `TerminalCommandType_t` and fields in `Core/Inc/terminal_parser.h`.
2. Parsing and range validation in `Core/Src/terminal_parser.c`.
3. DISARM allow/deny policy and execution in `Core/Src/command_handler.c`.
4. Dispatcher/protocol encoding for F411-bound commands.
5. Firmware `help` output.
6. `README.md` command reference.
7. GUI command generation, confirmation parsing, and controls.
8. Bridge behavior only when transport semantics actually change.

Do not silently change an existing command's meaning, range, case handling, or
line termination. Prefer backward-compatible aliases when a deployed command
must evolve.

Direct motor raw commands (`FL`, `FR`, `RL`, `RR`) must remain motor-specific.
Never broadcast a direct command unless the explicit target is `ALL` or the
high-level command is defined as broadcast.

## Telemetry and Logging Contracts

Firmware output is consumed by regex and key/value parsers in `earendil.py`.
Treat machine-readable lines as an API.

- Preserve stable record markers such as `[TEL][FL]`, `MPU_IMU,`,
  magnetometer records, `PC_LINK,`, `[PC_LINK]`, `[MODE]`, and `[ARM_RX]`.
- Prefer additive `KEY:VALUE` fields rather than renaming/removing existing
  fields.
- Keep one logical record on one CR/LF-terminated line.
- Do not insert free-form text inside a machine-readable payload.
- Use integer-scaled units where the current protocol expects them, and state
  the scale in code comments and documentation.
- If a telemetry/log format must change, update every GUI parser and associated
  freshness/status logic in the same task.
- Do not log heartbeat commands on every cycle; preserve quiet-link behavior.

## I2C, IMU, and Magnetometer Rules

- MPU and magnetometer share I2C1. Recovery for one device can affect the other.
- Keep I2C operations bounded with explicit timeouts.
- Do not run a full bus scan automatically in the fast path. Startup probing
  should remain limited to known addresses; `i2cscan` is an explicit diagnostic.
- Sensor streaming and reconnection must remain non-blocking state machines.
- After bus/peripheral recovery, invalidate or reset cached initialization state
  for all affected sensor modules.
- Do not repeatedly reinitialize a healthy sensor merely to obtain another
  sample. Fix state, timing, or error handling at the actual failure boundary.
- Preserve diagnostic counters and HAL error reporting when changing recovery.

## GUI Rules

- Keep network I/O event-driven through `QTcpSocket` and Qt signals.
- Never block the GUI thread with `sleep`, busy-waiting, serial reads, or socket
  loops. Use `QTimer` for command pacing and timeouts.
- Preserve the pressed-key/state model that allows motion keys and Shift/Ctrl
  speed adjustment to work simultaneously.
- Do not let auto-repeat generate uncontrolled motion-command floods.
- Keep emergency stop/brake controls available regardless of the active tab or
  settings dialog.
- Keep telemetry parsers tolerant of optional logger prefixes, but do not make
  them so broad that unrelated lines are accepted as telemetry.
- Bound TCP RX/TX buffers and text-console history.
- GUI state may show `UNKNOWN`, `STALE`, or pending states when confirmation is
  missing; do not fabricate a healthy/connected state.
- Keep root `earendil_logo.png` loading relative to the script location rather
  than the current working directory when touching asset loading.

## Bridge Rules

- Keep TCP and serial forwarding byte-transparent except for the intentional
  safety-stop injection.
- Preserve full-write loops; a partial TCP or serial write is not success.
- Protect shared serial/client objects with the existing lock/event model.
- Do not add silent multi-client command interleaving.
- Reject a client when the serial link is unavailable unless the task explicitly
  redesigns connection semantics and preserves safety.
- Preserve `TCP_NODELAY` behavior for low-latency control.
- Log enough context to diagnose bind, allow-list, serial-open, disconnect, and
  safety-stop failures without exposing secrets or flooding logs.
- Do not hardcode a developer's IP address or `/dev/ttyACM*` path.

## Documentation Rules

Update `README.md` when a change affects any of these:

- command syntax or range;
- pin/UART mapping;
- startup order or operating mode behavior;
- telemetry/log format;
- GUI/bridge deployment;
- required dependencies or invocation;
- safety, watchdog, or timeout behavior.

Do not rewrite roadmap/history files as if they were current implementation
specifications unless the task explicitly asks for roadmap maintenance.

## Validation Policy

Do **not** build, flash, connect to hardware, install packages, or regenerate
CubeMX code unless the user explicitly asks for that action.

For firmware tasks, the generated `Debug/makefile` contains machine-specific
absolute paths and is not a portable verification command. Never report a
firmware build as successful unless it was actually run in a valid toolchain.

For Python-only tasks, a side-effect-free syntax check may be used when
appropriate:

```bash
python3 -m py_compile earendil.py tcp_uart_bridge.py
```

Do not launch the GUI, open a serial device, bind a production port, or send
motor commands merely as a validation step.

When hardware verification is required, provide exact commands and expected
observations, but clearly separate code inspection from user-performed build,
flash, and bench testing.

## Change Workflow

1. Read the relevant module, its header, direct callers, and related README
   section before editing.
2. Identify the invariant or contract responsible for the behavior.
3. Make the smallest coherent change across all affected layers.
4. Re-check safety gates, callback context, buffer lifetime, and protocol
   compatibility.
5. Inspect the final diff for unrelated edits and generated artifacts.
6. Report what changed, why it changed, and what remains unverified.

Do not modify unrelated files to make the repository appear cleaner. Do not
silently delete diagnostics, comments, compatibility paths, or safety checks.

## Required Final Report for Coding Tasks

At completion, provide:

- files changed;
- functions/classes or sections changed;
- root cause or design rationale;
- behavior preserved, especially safety and protocol behavior;
- validation actually performed;
- build/flash/hardware tests not performed;
- any remaining risks or exact user test steps.

Never claim successful flashing, physical motor behavior, sensor stability, or
network reachability from source inspection alone.
