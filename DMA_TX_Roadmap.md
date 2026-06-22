# STM32H723ZG Rover Main Controller — Motor UART DMA TX Roadmap

## Purpose

This roadmap is for converting **motor UART TX** from blocking transmit to **DMA-based non-blocking transmit**.

Current intended final architecture:

```text
Motor UART RX  -> DMA + IDLE detection
Motor UART TX  -> DMA + per-motor TX state
USART3 terminal -> existing terminal system, unchanged unless explicitly needed
```

This roadmap must be executed carefully because the current **motor UART RX DMA system is already working** and must not be broken.

---

## Hard Rules

Do **not** modify or break:

```text
motor UART RX DMA
HAL_UARTEx_ReceiveToIdle_DMA()
HAL_UARTEx_RxEventCallback()
DMA RX restart logic
UART error callback policy
.dma_buffer RX buffer placement
MPU configuration
USART3 terminal RX
terminal parser behavior
command_handler behavior
control_mode behavior
```

Do **not** implement Stage 7 features in this task:

```text
ACK parser
STATUS parser
FAULT parser
heartbeat
watchdog
e-stop
motor state table
new feedback protocol
```

This roadmap is only for **motor UART DMA TX**.

---

## Current Motor UARTs

Motor UART peripherals:

```text
USART2
UART4
UART5
UART7
```

Terminal UART:

```text
USART3
```

Existing motor UART RX DMA streams:

```text
UART4_RX  -> DMA1_Stream0
UART5_RX  -> DMA1_Stream1
UART7_RX  -> DMA1_Stream2
USART2_RX -> DMA1_Stream3
```

TX DMA must use separate DMA streams and must not conflict with these RX streams.

---

## Stage TX-0 — Audit Current Motor TX Path

### Goal

Find every place where motor UART TX currently happens.

Search for:

```text
HAL_UART_Transmit(
MotorDispatcher_Send
MotorDispatcher_SendRaw
MotorLink_Send
MotorLink_Transmit
```

### Determine

```text
Which file sends motor commands?
Which functions are used for raw commands?
Which commands use motor UART TX?
Is USART3 terminal/logger TX separate from motor UART TX?
```

### Commands That Must Continue Working

```text
help
mode
mode rpm
mode pwm

f0..f200
b0..b200
r0..r200
l0..l200

fd0..fd255
bd0..bd255
rd0..rd255
ld0..ld255

stop
brake
identify
status
```

### Acceptance Criteria

- All blocking motor UART TX calls are identified.
- USART3 terminal/logger TX is clearly separated from motor UART TX.
- No code behavior has been changed yet unless required for the audit.

---

## Stage TX-1 — Configure TX DMA in CubeMX

### Goal

Add TX DMA channels for all four motor UARTs.

Required TX DMA requests:

```text
USART2_TX
UART4_TX
UART5_TX
UART7_TX
```

### Recommended DMA Settings

```text
Direction: Memory to Peripheral
Mode: Normal
Peripheral increment: Disabled
Memory increment: Enabled
Data width: Byte
Priority: Medium
```

Recommended priority policy:

```text
Motor UART RX DMA: High
Motor UART TX DMA: Medium
```

Reason: motor feedback RX should have priority over short command TX.

### Required Checks

After CubeMX generation, confirm:

```text
huart2.hdmatx is linked
huart4.hdmatx is linked
huart5.hdmatx is linked
huart7.hdmatx is linked

TX DMA IRQ handlers exist
TX DMA NVIC interrupts are enabled
Existing RX DMA streams are unchanged
```

### Acceptance Criteria

- Project builds after CubeMX TX DMA configuration.
- RX DMA configuration is still intact.
- No motor command behavior has changed yet.

---

## Stage TX-2 — Design DMA-Safe TX Buffers

### Goal

Create safe persistent TX buffers for DMA transmission.

DMA TX must not use local stack buffers.

Bad example:

```c
char buf[16];
snprintf(buf, sizeof(buf), "f100");
HAL_UART_Transmit_DMA(&huart2, (uint8_t*)buf, strlen(buf));
```

This is unsafe because DMA may still be reading the buffer after the function returns.

### Required Per-Motor State

Each motor UART must have its own:

```text
TX buffer
TX length
busy flag
pending buffer
pending length
pending flag
```

Suggested structure:

```c
typedef struct
{
    UART_HandleTypeDef *huart;

    uint8_t txBuffer[64];
    uint16_t txLen;
    bool busy;

    uint8_t pendingBuffer[64];
    uint16_t pendingLen;
    bool pending;
} MotorTxDmaChannel_t;
```

### Memory Placement

Preferred:

```text
.dma_buffer section
32-byte aligned
RAM_D2 / non-cacheable MPU region
```

Example:

```c
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t fl_tx_buffer[64];
```

If D-Cache is currently disabled, DMA TX may work even without this.  
However, for future D-Cache safety, TX buffers should be DMA-safe just like RX buffers.

### Acceptance Criteria

- No local TX buffer is used for DMA.
- Every motor UART has persistent TX storage.
- TX buffers are safe for DMA lifetime.
- TX buffers are preferably in DMA-safe memory.

---

## Stage TX-3 — Create `motor_tx_dma.c/h`

### Goal

Create a dedicated module for motor UART DMA TX.

New files:

```text
motor_tx_dma.c
motor_tx_dma.h
```

### Required Public API

Suggested API:

```c
void MotorTxDma_Init(void);

bool MotorTxDma_Send(MotorId_t motor, const char *cmd);
bool MotorTxDma_SendAll(const char *cmd);

void MotorTxDma_OnTxComplete(UART_HandleTypeDef *huart);
void MotorTxDma_OnTxError(UART_HandleTypeDef *huart);
```

### Architecture Rule

`HAL_UART_Transmit_DMA()` must only be called inside `motor_tx_dma.c`.

Other modules should call:

```c
MotorTxDma_Send(...)
MotorTxDma_SendAll(...)
```

They should not call HAL DMA TX directly.

### Acceptance Criteria

- `motor_tx_dma.c/h` exists.
- Module compiles.
- No behavior change yet unless connected in later stages.
- `main.c` does not contain TX DMA state logic.

---

## Stage TX-4 — Implement One-UART Prototype

### Goal

Test DMA TX on only one motor UART first.

Recommended first UART:

```text
USART2 / FL
```

### Test Commands

```text
mode rpm
f100
f205
stop
brake
identify
status
```

### Check

```text
HAL_UART_Transmit_DMA() returns HAL_OK
TX DMA interrupt fires
HAL_UART_TxCpltCallback() is called
TX busy flag clears
Pending command is handled
Motor UART RX DMA still works
```

### Acceptance Criteria

- One motor UART transmits using DMA.
- Main loop does not block during TX.
- TX complete callback works.
- RX DMA still works.

Do not proceed to four UARTs until this stage is stable.

---

## Stage TX-5 — Add TX Complete Callback Router

### Goal

Route HAL TX complete events to the DMA TX module.

Required callback:

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    MotorTxDma_OnTxComplete(huart);
}
```

### Important

There must be only one definition of:

```c
HAL_UART_TxCpltCallback
```

in the whole project.

If the project already has this callback, extend it safely instead of creating a duplicate.

### Required Behavior

When TX completes:

```text
Find the matching motor UART channel.
Set busy = false.
If pending exists:
    move pending -> active tx buffer
    clear pending flag
    start next HAL_UART_Transmit_DMA()
    set busy = true
```

### Acceptance Criteria

- No duplicate HAL callback definitions.
- TX complete clears the correct motor channel.
- Pending command starts automatically after active TX completes.

---

## Stage TX-6 — Implement Busy / Pending System

### Goal

Prevent `HAL_BUSY` problems and command loss.

### Required Behavior

If UART TX is idle:

```text
copy command to txBuffer
start HAL_UART_Transmit_DMA()
busy = true
```

If UART TX is busy:

```text
copy command to pendingBuffer
pending = true
do not block
do not call HAL_Delay()
do not spin-wait
```

When current TX completes:

```text
busy = false
if pending:
    pendingBuffer -> txBuffer
    pending = false
    start DMA TX
    busy = true
```

### Minimum Policy

One pending slot per motor UART is enough for the first implementation.

### Acceptance Criteria

- No blocking wait exists in TX path.
- `HAL_BUSY` does not cause command loss silently.
- One pending command per motor UART is supported.

---

## Stage TX-7 — Add Stop/Brake Priority Policy

### Goal

Make safety-critical commands harder to lose.

Safety-critical commands:

```text
stop
brake
```

Brake command payload:

```text
x
```

### Required Policy

If a normal command arrives while TX is busy:

```text
store/overwrite pending with the newest normal command
```

If `stop` or `brake` arrives while TX is busy:

```text
overwrite pending immediately
pending becomes stop or brake
```

Example:

```text
f100 active TX
f120 pending
brake arrives
pending becomes x
```

Do not abort active TX in the first implementation unless explicitly required later.

### Acceptance Criteria

- Stop/brake do not get dropped when TX is busy.
- Stop/brake can overwrite normal pending command.
- No blocking behavior is introduced.

---

## Stage TX-8 — Expand to Four Motor UARTs

### Goal

Enable DMA TX for all motor UARTs:

```text
USART2
UART4
UART5
UART7
```

### Important Motor Mapping Check

Before enabling all four, verify the correct mapping:

```text
FL -> USART2
FR -> UART4
RL -> UART5 or UART7
RR -> UART7 or UART5
```

The code, wiring, and roadmap must agree.

### Test Commands

```text
mode rpm
f100
b100
r100
l100
stop
brake
identify
status

mode pwm
fd100
fd260
```

### Acceptance Criteria

- All four motor UARTs use DMA TX.
- Each motor UART has independent busy/pending state.
- Commands are sent to correct motor UARTs.
- RX DMA still works on all four motor UARTs.

---

## Stage TX-9 — Integrate with Motor Dispatcher

### Goal

Replace motor UART blocking TX calls with DMA TX module calls.

Old motor TX path:

```c
HAL_UART_Transmit(...)
```

New motor TX path:

```c
MotorTxDma_Send(motor, cmd);
```

or:

```c
MotorTxDma_SendAll(cmd);
```

### Behavior Must Stay Same

Do not change parser or command behavior.

These mappings must remain:

```text
f205  -> f200
fd260 -> f255
brake -> x
mode rpm -> mode rpm
mode pwm -> mode pwm
identify -> identify
status -> status
```

Mode restrictions must remain:

```text
RPM mode:
  f/b/r/l allowed
  fd/bd/rd/ld rejected

PWM mode:
  fd/bd/rd/ld allowed
  f/b/r/l rejected
```

Invalid-mode commands must not transmit anything.

### Acceptance Criteria

- Motor UART TX no longer uses blocking `HAL_UART_Transmit()`.
- USART3 logger/terminal may still use blocking TX.
- Existing command behavior is unchanged.
- Project builds.

---

## Stage TX-10 — Verify DMA-Safe Memory in `.map`

### Goal

Verify TX buffers are placed in safe memory.

Check map file for TX buffer symbols:

```text
fl_tx_buffer
fr_tx_buffer
rl_tx_buffer
rr_tx_buffer
pending buffers
```

Preferred address range:

```text
0x3000....
```

This indicates RAM_D2 / DMA-safe region.

### Acceptance Criteria

- TX DMA buffers are in DMA-safe memory.
- RX DMA buffers remain in DMA-safe memory.
- No TX buffer is accidentally placed in stack/local memory.

---

## Stage TX-11 — Stress Test

### Goal

Test fast command sequences and pending handling.

### RPM Stress Test

```text
mode rpm
f100
f120
f140
f160
f180
f205
stop
brake
```

### PWM Stress Test

```text
mode pwm
fd100
fd120
fd140
fd260
stop
brake
```

### Common Command Test

```text
identify
status
mode rpm
mode pwm
help
```

### Check

```text
No main loop blocking
No HAL_BUSY spam
No command crash
Stop/brake not lost
Pending behavior is correct
TX complete callback count is reasonable
RX DMA continues receiving responses
```

### Acceptance Criteria

- Stress commands do not break TX state.
- Stop/brake remain reliable.
- RX DMA continues working.
- Terminal remains responsive.

---

## Stage TX-12 — Prepare for Stage 7 Feedback/Safety

### Goal

Make DMA TX ready for later ACK/status/fault integration.

After DMA TX is stable, Stage 7 can connect:

```text
TX sent
TX complete
ACK wait starts
ACK received
STATUS received
FAULT received
timeout handled
```

Future Stage 7 modules:

```text
ACK parser
STATUS parser
FAULT parser
motor state table
link timeout
heartbeat
e-stop
watchdog
```

Do not implement these in the DMA TX task.

### Acceptance Criteria

- DMA TX module exposes enough state for future feedback logic.
- TX complete can be tracked per motor.
- Safety commands have priority policy.

---

## Final Acceptance Criteria

DMA TX implementation is complete only when:

```text
All four motor UARTs use HAL_UART_Transmit_DMA()
No blocking motor UART HAL_UART_Transmit() calls remain
USART3 logger/terminal behavior is preserved
Motor UART RX DMA remains functional
TX complete callback works
Each motor has independent busy/pending state
Stop/brake priority is implemented
TX buffers are DMA-safe
Project builds successfully
Stress tests pass
```

---

## Final Target Architecture

```text
terminal command
    ↓
terminal_parser
    ↓
command_handler
    ↓
motion_controller / motor_dispatcher
    ↓
motor_tx_dma
    ↓
HAL_UART_Transmit_DMA()
    ↓
USART2 / UART4 / UART5 / UART7
```

RX path remains:

```text
USART2 / UART4 / UART5 / UART7
    ↓
RX DMA + IDLE
    ↓
motor_uart_dma
```

---

## Notes for the Coding Agent

- Work stage by stage.
- Do not combine DMA TX with Stage 7 feedback work.
- Do not refactor unrelated systems.
- Do not touch RX DMA unless required for a compile error.
- Preserve all terminal command behavior.
- Report exactly which stage was completed and what was tested.
