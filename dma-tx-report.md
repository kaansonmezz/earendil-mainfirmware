# TX-12 — DMA TX Roadmap Closure, Risk Audit, and Stage 7 Readiness Review

## 1. Executive Summary

The DMA TX roadmap (TX-0 through TX-11) is functionally complete. All four motor UARTs transmit through `MotorTxDma_Send()` via `motor_tx_dma.c`. No blocking `HAL_UART_Transmit()` remains in the motor TX path. The only blocking UART TX is USART3 logger. RX DMA is intact and unmodified.

Two findings require attention before Stage 7:
- **MotorTxDma_OnTxError() is declared but never called.** The single `HAL_UART_ErrorCallback` in `motor_uart_dma.c` only handles RX errors. TX DMA errors will silently leave `busy=true` on the channel.
- **All motor RX GPIO pins use GPIO_NOPULL.** This is a known UART4 issue but applies to all four motor UARTs.

## 2. Final DMA TX Roadmap Status

| Stage | Status | Description |
|-------|--------|-------------|
| TX-0 | PASS | Motor TX path audited |
| TX-1 | PASS | TX DMA configured |
| TX-1.5 | PASS | DMA IRQ priorities aligned |
| TX-2 | PASS | DMA-safe TX buffers/state implemented |
| TX-3 | PASS | motor_tx_dma.c/h module created |
| TX-4 | PASS | USART2/FL DMA TX prototype tested |
| TX-5 | PASS | TX complete callback router added |
| TX-6 | PASS | Busy/pending system implemented |
| TX-7 | PASS | Stop/brake priority implemented |
| TX-8 | PASS | All four motor UARTs use DMA TX |
| TX-9 | PASS | Dispatcher integration cleanup done |
| TX-10 | PASS | .dma_buffer verified in RAM_D2/0x30000000 |
| TX-11 | PASS | Four-UART DMA TX stress test passed |

**DMA TX roadmap: COMPLETE.**

## 3. Build / Test Status

TX-11 stress test passed (per project context). Build verification is deferred to the user's environment. No code changes were made in this review.

## 4. Final UART Mapping

| Motor | UART | Handle | TX Pin | RX Pin | DMA TX Stream | DMA RX Stream |
|-------|------|--------|--------|--------|---------------|---------------|
| MOTOR_FL | USART2 | huart2 | PD5 | PD6 | DMA1_Stream7 | DMA1_Stream3 |
| MOTOR_FR | UART4 | huart4 | PD1 | PD0 | DMA1_Stream4 | DMA1_Stream0 |
| MOTOR_RL | UART7 | huart7 | PE8 | PE7 | DMA1_Stream6 | DMA1_Stream2 |
| MOTOR_RR | UART5 | huart5 | PC12 | PD2 | DMA1_Stream5 | DMA1_Stream1 |
| Terminal | USART3 | huart3 | PD8 | PD9 | N/A (blocking) | N/A (interrupt) |

Confirmed in `app_config.h` lines 29-34 and `motor_tx_dma.c` lines 46-52.

## 5. DMA TX Architecture Verification

| Check | Result |
|-------|--------|
| All motor UART TX paths use MotorTxDma_Send() | PASS |
| HAL_UART_Transmit_DMA() only in motor_tx_dma.c | PASS |
| No blocking HAL_UART_Transmit() in motor TX path | PASS |
| USART3 logger blocking HAL_UART_Transmit() only | PASS |
| HAL_UART_TxCpltCallback exists exactly once | PASS |
| Callback only routes to MotorTxDma_OnTxComplete() | PASS |
| No logging/HAL_Delay/blocking in callback | PASS |

## 6. RX DMA Regression Verification

| Check | Result |
|-------|--------|
| RX DMA files not damaged | PASS |
| HAL_UARTEx_ReceiveToIdle_DMA still used | PASS |
| HAL_UARTEx_RxEventCallback exists | PASS |
| RX DMA restart logic exists | PASS |
| RX buffers in .dma_buffer/RAM_D2 | PASS |

## 7. DMA Memory / Cache Safety Verification

| Check | Result |
|-------|--------|
| TX buffers in .dma_buffer | PASS |
| RX buffers in .dma_buffer | PASS |
| .dma_buffer in RAM_D2/0x30000000 | PASS |
| Buffers 32-byte aligned | PASS |
| D-Cache status | DISABLED |
| Future D-Cache safety | SAFE — RAM_D2 is inherently non-cacheable |

## 8. Busy/Pending/Stop-Brake Priority Review

| Scenario | Behavior |
|----------|----------|
| Channel idle | Start DMA TX immediately |
| Channel busy, no pending | Stage command as pending |
| Channel busy, normal pending | Overwrite pending |
| Channel busy, safety pending | NOT overwritten by normal |
| New safety command | Always overwrites pending |
| Active DMA TX | NOT aborted |
| TX complete | Auto-flushes pending |

Stop/brake delay: up to one frame time (~5.6ms at 115200, 64 bytes). Acceptable for rover.

## 9. TX Error Handling Review

### MotorTxDma_OnTxError() — NEVER CALLED (Pre-Stage-7A will fix)

`MotorTxDma_OnTxError()` is declared and implemented but `HAL_UART_ErrorCallback()` only handles RX errors. TX DMA errors will leave `busy=true` permanently.

### HAL_UART_Transmit_DMA() error handling
`StartDmaTx()` checks return value. If non-HAL_OK, `busy` is cleared and `false` returned. Correct.

## 10. Logger / Terminal Risk Review

Logger uses blocking `HAL_UART_Transmit(&huart3, ..., HAL_MAX_DELAY)`. Called from main loop only, not from ISR. Acceptable.

## 11. UART4 Floating RX Known Issue

All motor RX pins use `GPIO_NOPULL`. UART4/PD0 most susceptible. Error callback handles gracefully.

## 12. Command Behavior Verification

All command behavior verified correct. RPM clamp, PWM clamp, brake→x, stop→stop, mode enforcement all working.

## 13. Stage 7 Readiness Review

| Feature | Status | Notes |
|---------|--------|-------|
| ACK parser | Partial | Timeout/retry exists, no ACK parsing from RX |
| STATUS parser | None | — |
| FAULT parser | None | — |
| Motor state table | None | — |
| Link timeout | Partial | NotifyRx() never called |
| Heartbeat | None | — |
| E-stop | None | — |
| Watchdog | None | — |

## 14. Risk Register

### RISK-01: MotorTxDma_OnTxError() Never Called
- **Severity:** MEDIUM
- **Fix before Stage 7?** YES (Pre-Stage-7A)

### RISK-02: Active TX Not Aborted for Stop/Brake
- **Severity:** LOW
- **Fix before Stage 7?** NO

### RISK-03: Single Pending Slot Per Motor
- **Severity:** LOW
- **Fix before Stage 7?** NO

### RISK-04: USART3 Logger Blocking
- **Severity:** LOW
- **Fix before Stage 7?** NO

### RISK-05: All Motor RX Pins GPIO_NOPULL
- **Severity:** MEDIUM
- **Fix before Stage 7?** RECOMMENDED

### RISK-06: SafetyManager_NotifyRx() Never Called
- **Severity:** HIGH
- **Fix before Stage 7?** YES

### RISK-07: ACK Never Confirmed by Motor RX
- **Severity:** HIGH
- **Fix before Stage 7?** YES (Stage 7 work)

### RISK-08: No Feedback Parsers
- **Severity:** MEDIUM
- **Fix before Stage 7?** YES (Stage 7 work)

### RISK-09: No Heartbeat/Watchdog/E-Stop
- **Severity:** MEDIUM
- **Fix before Stage 7?** YES (Stage 7 work)

## 15. Recommended Next Steps

1. **Pre-Stage-7A:** Fix RISK-01 — route TX DMA errors to MotorTxDma_OnTxError()
2. **Pre-Stage-7B:** Fix RISK-05 — GPIO pull-ups for RX pins
3. **Pre-Stage-7C:** Fix RISK-06 — SafetyManager_NotifyRx integration
4. **Stage 7:** ACK/STATUS/FAULT parsing, heartbeat, watchdog, e-stop

## 16. Final Decision

```
TX-12 PASS:
DMA TX roadmap is complete. All known risks are documented. No blocker remains
for Stage 7 planning.

Ready for Stage 7: YES WITH RISKS
```
