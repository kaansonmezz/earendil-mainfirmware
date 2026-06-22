# STM32H723ZG Rover Main Controller — HAL + UART DMA Roadmap

## 1. Project Objective

This firmware will be developed for the **NUCLEO-H723ZG** as the rover’s **main real-time controller** using the **STM32 HAL library**.

The first development target is:

- Stable **HAL-based project foundation**
- **USART3** as the PC terminal / command interface through **ST-LINK Virtual COM Port**
- Four motor-controller UART channels:
  - `USART2`
  - `UART4`
  - `UART5`
  - `UART7`
- **RX communication on motor UARTs using DMA + IDLE line detection**
- Clear modular architecture suitable for:
  - motor command routing,
  - ACK/status feedback,
  - e-stop logic,
  - command parser,
  - later heartbeat and fail-safe mechanisms.

---

## 2. Current Project State

### 2.1 Clock Configuration — Completed

The project currently uses:

| Clock Domain | Frequency |
|---|---:|
| CPU / SYSCLK | 520 MHz |
| HCLK / AHB | 260 MHz |
| APB1 / APB2 / APB3 / APB4 | 130 MHz |

Clock source:

- `HSE Bypass`
- `PLL1`
- `SYSCLK source = PLLCLK`

This is suitable for the H723ZG project and leaves enough system performance margin for multi-UART DMA, command handling, state management, and future communication extensions.

---

### 2.2 UART Assignment — Completed

| Function | Peripheral | TX Pin | RX Pin |
|---|---|---:|---:|
| Motor Driver 1 | `USART2` | `PD5` | `PD6` |
| Motor Driver 2 | `UART4` | `PD1` | `PD0` |
| Motor Driver 3 | `UART5` | `PC12` | `PD2` |
| Motor Driver 4 | `UART7` | `PE8` | `PE7` |
| PC Terminal / ST-LINK VCP | `USART3` | `PD8` | `PD9` |

Design note:

- `USART3` is reserved for the terminal/serial monitor over ST-LINK VCP.
- Motor UARTs avoid this terminal channel.
- The selected motor UART pins also preserve `PA11/PA12` for future USB CDC use if needed.

---

### 2.3 DMA Assignment — Completed in CubeMX

The four motor UART RX channels are already mapped to separate DMA streams:

| UART RX | DMA Stream |
|---|---|
| `UART4_RX` | `DMA1_Stream0` |
| `UART5_RX` | `DMA1_Stream1` |
| `UART7_RX` | `DMA1_Stream2` |
| `USART2_RX` | `DMA1_Stream3` |

Current DMA configuration:

- Direction: `Peripheral to Memory`
- Mode: `Normal`
- Peripheral increment: `Disabled`
- Memory increment: `Enabled`
- Data width: `Byte`
- Priority: `High`

---

### 2.4 Interrupt Configuration — Completed in CubeMX

The following interrupt groups are enabled:

#### Motor UART IRQs
- `USART2_IRQn`
- `UART4_IRQn`
- `UART5_IRQn`
- `UART7_IRQn`

#### DMA IRQs
- `DMA1_Stream0_IRQn`
- `DMA1_Stream1_IRQn`
- `DMA1_Stream2_IRQn`
- `DMA1_Stream3_IRQn`

#### Terminal UART IRQ
- `USART3_IRQn`

These are required for:

- DMA RX transfer events
- UART IDLE-line detection
- Terminal receive logic

---

## 3. Immediate Next Milestone: Basic Firmware Bring-Up

Before implementing full DMA logic, verify that the generated HAL project boots correctly and that the terminal UART works.

### 3.1 Build and Flash the Generated Project

Tasks:

- Open the project in **STM32CubeIDE**
- Build without warnings that indicate configuration issues
- Flash to NUCLEO-H723ZG
- Confirm that the board runs without entering `Error_Handler()`

Acceptance criteria:

- Project compiles
- Flash succeeds
- MCU does not halt unexpectedly after boot

---

### 3.2 USART3 Terminal TX Test

Add a simple boot message:

```c
const char *msg = "H723 HAL project started\r\n";
HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
```

Recommended placement:

- After all peripheral initialization calls
- Inside `/* USER CODE BEGIN 2 */`

Acceptance criteria:

- PlatformIO serial monitor opens the ST-LINK VCP port
- Message is received cleanly at `115200 baud`

---

### 3.3 USART3 Terminal RX Test

Create a minimal receive mechanism for terminal commands.

Initial test strategy:

- Use interrupt-based byte reception or Receive-to-IDLE interrupt reception
- Echo received terminal text back to the serial monitor

Acceptance criteria:

- PC sends:
  ```text
  hello
  ```
- Board returns:
  ```text
  RX: hello
  ```

Recommended priority:

- Terminal UART should remain lower priority than motor feedback handling.

---

## 4. Phase 1 — Motor UART DMA RX Foundation

### 4.1 Define Dedicated DMA RX Buffers

Create one raw DMA buffer per motor UART:

```c
#define MOTOR_DMA_RX_BUFFER_SIZE 128

uint8_t usart2_rx_dma_buffer[MOTOR_DMA_RX_BUFFER_SIZE];
uint8_t uart4_rx_dma_buffer[MOTOR_DMA_RX_BUFFER_SIZE];
uint8_t uart5_rx_dma_buffer[MOTOR_DMA_RX_BUFFER_SIZE];
uint8_t uart7_rx_dma_buffer[MOTOR_DMA_RX_BUFFER_SIZE];
```

Recommended future upgrade:

- Place DMA buffers in a dedicated DMA-safe memory section once the cache/memory architecture is finalized.

---

### 4.2 Start RX DMA with IDLE Detection

After peripheral initialization:

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_rx_dma_buffer, MOTOR_DMA_RX_BUFFER_SIZE);
HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma_buffer, MOTOR_DMA_RX_BUFFER_SIZE);
HAL_UARTEx_ReceiveToIdle_DMA(&huart5, uart5_rx_dma_buffer, MOTOR_DMA_RX_BUFFER_SIZE);
HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_rx_dma_buffer, MOTOR_DMA_RX_BUFFER_SIZE);
```

Disable half-transfer interrupts initially:

```c
__HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
__HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
__HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
```

Reason:

- The first architecture should respond to complete variable-length packets detected by IDLE or DMA completion.
- Half-transfer callbacks are not useful for the first packet-based ASCII protocol implementation.

---

### 4.3 Implement a Unified RX Event Callback

Implement:

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
```

Inside the callback:

- Detect which UART generated the event
- Copy the received bytes to a software-level processing buffer
- Mark that motor’s RX message as ready
- Restart DMA reception for that UART

Do **not**:

- parse full commands inside the callback
- call blocking transmit heavily inside the callback
- execute motor control logic inside the callback

Acceptance criteria:

- Sending a short UART line from an external source to each motor UART triggers the callback
- `Size` reflects received byte count
- DMA restarts correctly and receives multiple packets repeatedly

---

## 5. Phase 2 — DMA RX Validation for All Four Motor Channels

### 5.1 Single-Channel Test First

Start with `USART2_RX` only.

Test sequence:

1. Connect an external USB-UART adapter to:
   - H723 `PD6` RX
   - GND common
2. Send:
   ```text
   TEST_USART2
   ```
3. Print the received bytes to `USART3` terminal

Acceptance criteria:

- Correct data is printed
- Callback triggers once per IDLE gap
- Repeated packets are received without lockup

---

### 5.2 Expand to UART4, UART5, UART7

Test all four channels one by one.

Suggested printable debug tags:

```text
[USART2_RX] ...
[UART4_RX] ...
[UART5_RX] ...
[UART7_RX] ...
```

Acceptance criteria:

- Each UART RX channel receives independently
- No stream overlap
- No channel blocks another
- No hard fault after repeated tests

---

### 5.3 Multi-Channel Concurrency Test

Feed data to multiple UART RX channels in a short time window.

Acceptance criteria:

- All RX events are captured
- No DMA stream conflict
- No missed packets at the intended data rate
- Terminal output clearly identifies source UART

---

## 6. Phase 3 — Cache and DMA Memory Correctness on STM32H7

STM32H723 uses Cortex-M7 cache architecture. DMA writes to RAM, and the CPU later reads that RAM. This can cause cache coherency issues if buffer placement and cache handling are not deliberately managed.

### 6.1 First Practical Approach

During early bring-up:

- Keep buffers simple
- Verify whether RX DMA behaves consistently
- Avoid prematurely optimizing memory layout until functionality is confirmed

### 6.2 Required Final Approach

Before integrating the final rover protocol, choose one of these approaches:

#### Option A — Non-cacheable MPU DMA region
Recommended long-term.

- Create a dedicated memory region for DMA buffers
- Configure MPU region as:
  - non-cacheable
  - shareable if required
- Place UART RX DMA buffers in this section through linker configuration

#### Option B — Explicit cache maintenance
Alternative.

- Invalidate D-cache lines covering RX DMA buffers before CPU reads them
- Clean D-cache for TX DMA buffers before DMA reads them

### 6.3 Final Decision

Recommended for this project:

> Use a dedicated **DMA-safe non-cacheable memory region** for communication buffers.

Acceptance criteria:

- RX contents remain correct over long test periods
- No “first byte only”, stale, or partially outdated buffer behavior
- Multi-UART DMA receive remains stable

---

## 7. Phase 4 — UART Communication Abstraction Layer

After raw DMA RX is confirmed, move logic out of `main.c`.

### 7.1 Recommended File Structure

```text
Core/
├── Inc/
│   ├── app_main.h
│   ├── terminal_uart.h
│   ├── motor_uart_dma.h
│   ├── ring_buffer.h
│   ├── command_parser.h
│   ├── motor_dispatcher.h
│   ├── safety_manager.h
│   └── rover_types.h
│
└── Src/
    ├── app_main.c
    ├── terminal_uart.c
    ├── motor_uart_dma.c
    ├── ring_buffer.c
    ├── command_parser.c
    ├── motor_dispatcher.c
    ├── safety_manager.c
    └── rover_types.c
```

---

### 7.2 `motor_uart_dma` Module Responsibilities

Responsibilities:

- Start RX DMA for all motor UART channels
- Handle UART RX callback routing
- Maintain ready flags / queues
- Restart DMA cleanly
- Expose received packet retrieval API

Example future-facing functions:

```c
void MotorUartDma_Init(void);
void MotorUartDma_StartAllRx(void);
bool MotorUartDma_HasMessage(MotorId id);
uint16_t MotorUartDma_ReadMessage(MotorId id, uint8_t *dst, uint16_t max_len);
```

---

### 7.3 `terminal_uart` Module Responsibilities

Responsibilities:

- Terminal RX via USART3
- Terminal TX helper functions
- Print logging:
  - `[INFO]`
  - `[WARN]`
  - `[ERROR]`
- Terminal command receive buffer

Example functions:

```c
void Terminal_Init(void);
void Terminal_Print(const char *text);
void Terminal_Printf(const char *fmt, ...);
bool Terminal_HasLine(void);
bool Terminal_ReadLine(char *dst, uint16_t max_len);
```

---

## 8. Phase 5 — Rebuild Existing Rover Command Architecture

Once communication infrastructure is stable, port the proven application logic from the earlier Arduino-based architecture.

### 8.1 Terminal Command Parser

Initial commands:

```text
help
status
identify
stop
forward <pwm>
backward <pwm>
left <pwm>
right <pwm>
```

Later:

```text
arm <payload>
red on/off
yellow on/off
green on/off
```

Acceptance criteria:

- Commands from terminal are parsed correctly
- Malformed commands are rejected
- PWM range validation exists
- Commands do not block RX handling

---

### 8.2 Motor Dispatcher

Responsibilities:

- Convert application-level motion output to motor UART commands
- Send ASCII protocol messages to each F411 driver board
- Support later binary protocol replacement if desired

Initial output examples:

```text
f120
b120
stop
identify
```

Acceptance criteria:

- Each motor channel receives the intended command
- Stop command reliably reaches all four motor drivers
- The dispatcher can be called repeatedly without corrupting UART TX

---

### 8.3 Motion Controller

Responsibilities:

- Translate:
  - `forward`
  - `backward`
  - `left`
  - `right`
  - `stop`
- Into wheel-specific commands

Important design checkpoint:

- Re-confirm wheel direction conventions early
- Do not blindly port previous motion mappings without verifying real rover motion

Acceptance criteria:

- Real wheel behavior matches operator command semantics
- Tank turn behavior is verified physically

---

## 9. Phase 6 — Motor Feedback Protocol

Once motor TX dispatch and DMA RX both work, implement feedback handling.

### 9.1 Minimum Feedback Messages

Recommended baseline:

```text
ACK <cmd_id>
DONE <cmd_id>
FAULT <code>
STATUS <state>
```

### 9.2 H7 Responsibilities

- Receive feedback via DMA RX
- Associate message with source motor UART
- Parse message outside ISR context
- Update motor state table
- Print translated/debug information to terminal when needed

Example internal status table:

| Motor | Last ACK | Last Status | Last Fault | Last RX Time |
|---|---|---|---|---|
| FL | valid | running | none | timestamp |
| FR | valid | running | none | timestamp |
| RL | valid | running | none | timestamp |
| RR | valid | running | none | timestamp |

---

## 10. Phase 7 — Safety Architecture

### 10.1 E-stop Input

Tasks:

- Add E-stop GPIO input
- Implement debounce if required
- Define active state explicitly
- Introduce a system-level safety latch

Behavior:

- When E-stop becomes active:
  - immediately issue stop to all motor drivers
  - block motion commands
  - allow only status/help/safety-related terminal commands

---

### 10.2 Command Timeout / Heartbeat

Future system should support timeouts between:

- Ground station ↔ H7
- Jetson ↔ H7
- H7 ↔ each F411 motor driver

Recommended principle:

- No valid heartbeat or control message within timeout → enter safe stop state

---

### 10.3 Watchdog

Add independent watchdog after the main loop architecture is stable.

Responsibilities:

- Detect complete software lockups
- Reset MCU if application stops refreshing the watchdog

Do not add watchdog too early during bring-up, or it will make debugging harder.

---

## 11. Phase 8 — Code Quality and Reliability Work

### 11.1 Error Handling

Replace silent failures with deliberate diagnostics:

- UART HAL return values checked
- DMA start return values checked
- Error_Handler enriched with useful debug marker if possible
- Fault counters per UART

### 11.2 Avoid These Patterns

- Long blocking HAL delays in logic loops
- Heavy printing inside interrupts
- Full command parsing in callbacks
- Dynamic memory allocation where unnecessary
- Monolithic `main.c`

### 11.3 Testing Priorities

- Repeated DMA RX over thousands of packets
- Simultaneous motor feedback
- Noise and malformed packet handling
- Motor stop under communication failure
- Terminal commands during active motor feedback

---

## 12. Recommended Development Sequence

### Stage 0 — Already Complete
- [x] CubeMX project generated
- [x] Clock configured
- [x] Motor UARTs assigned
- [x] RX DMA streams assigned
- [x] NVIC IRQs enabled
- [x] Terminal USART3 enabled

### Stage 1 — Immediate Bring-Up
- [ ] Confirm project builds and flashes
- [ ] Terminal USART3 prints boot string
- [ ] Terminal RX echo works

### Stage 2 — DMA Receive Foundation
- [ ] Add four DMA RX buffers
- [ ] Start four `ReceiveToIdle_DMA()` receptions
- [ ] Implement RX event callback
- [ ] Restart DMA safely after each packet

### Stage 3 — DMA Validation
- [ ] Test USART2 RX only
- [ ] Test UART4 RX
- [ ] Test UART5 RX
- [ ] Test UART7 RX
- [ ] Test concurrent RX streams

### Stage 4 — H7 DMA Reliability
- [ ] Evaluate cache behavior
- [ ] Implement non-cacheable DMA buffer region or cache maintenance
- [ ] Run long-duration repeated RX tests

### Stage 5 — Firmware Architecture
- [ ] Create modular `terminal_uart`
- [ ] Create modular `motor_uart_dma`
- [ ] Create command parser
- [ ] Create motor dispatcher
- [ ] Create motion controller

### Stage 6 — Functional Rover Command Path
- [ ] Terminal command → parser
- [ ] Parser → motion controller
- [ ] Motion controller → motor dispatcher
- [ ] Motor dispatcher → 4 F411 boards
- [ ] Stop path tested

### Stage 7 — Feedback and Safety
- [ ] F411 ACK/status protocol
- [ ] H7 feedback parser
- [ ] Motor state table
- [ ] E-stop integration
- [ ] Communication timeout logic
- [ ] Watchdog

---

## 13. Near-Term Coding Order

The immediate coding order after this roadmap should be:

1. **USART3 boot print**
2. **USART3 terminal RX test**
3. **USART2 RX DMA single-channel test**
4. **All four motor UART RX DMA test**
5. **DMA callback refactor into `motor_uart_dma.c`**
6. **Terminal parser skeleton**
7. **Motor TX dispatcher skeleton**
8. **Existing rover control architecture port**

---

## 14. Definition of “Architecture Is Ready”

The H723 HAL project can be considered architecturally ready when:

- USART3 terminal accepts and processes text commands
- Four motor UARTs can:
  - transmit commands,
  - receive responses via DMA + IDLE
- No UART channel blocks the main loop
- Received motor feedback is parsed outside ISR context
- Stop command reaches all four drivers reliably
- E-stop logic blocks unsafe control
- Code is split into modules, not concentrated in `main.c`
- DMA memory/cache behavior has been deliberately addressed

---

## 15. Final Technical Direction

The project direction should remain:

> **HAL-based, modular, non-blocking central controller firmware with UART DMA feedback reception, clear safety logic, and a transport layer that can later evolve without rewriting the application layer.**
