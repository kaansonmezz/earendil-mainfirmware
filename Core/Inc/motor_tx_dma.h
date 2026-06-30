#ifndef MOTOR_TX_DMA_H
#define MOTOR_TX_DMA_H

#include <stdbool.h>
#include "main.h"
#include "rover_types.h"

/* ── Public API ─────────────────────────────────────────────────────────────
 * Motor UART DMA TX module.
 * Owns motor UART DMA TX state, a per-motor FIFO queue, and a single
 * DMA-safe active transmit buffer per channel.
 * All motor UART TX commands are routed through this module.
 * HAL_UART_TxCpltCallback routes TX-complete events to MotorTxDma_OnTxComplete(),
 * which drains the next queued frame automatically.
 * Safety commands (stop / x / brake) clear queued normal commands and are
 * transmitted next, after the active frame completes.
 * ─────────────────────────────────────────────────────────────────────────── */

void MotorTxDma_Init(void);

bool MotorTxDma_Send(MotorId_t motor, const char *cmd);
bool MotorTxDma_SendAll(const char *cmd);

void MotorTxDma_OnTxComplete(UART_HandleTypeDef *huart);
void MotorTxDma_OnTxError(UART_HandleTypeDef *huart);

bool MotorTxDma_IsBusy(MotorId_t motor);
bool MotorTxDma_HasPending(MotorId_t motor);

/* True when no motor TX channel is busy and none has a pending frame queued,
 * i.e. all motor UART TX paths are fully drained.  Used by the synchronized
 * control-mode switch to guarantee the previous frame (e.g. "stop") has been
 * fully clocked out before the next frame (e.g. "mode speed") is sent. */
bool MotorTxDma_AllIdle(void);

void MotorTxDma_CancelPending(void);  /* drop all queued (non-active) TX frames */

/* ── Optional debug getters ────────────────────────────────────────────────
 * Expose current queue depth / dropped count / high-watermark for status
 * and debugging.  Safe to call from main-loop context. */
uint8_t  MotorTxDma_GetQueueCount(MotorId_t motor);
uint32_t MotorTxDma_GetDroppedCount(MotorId_t motor);
uint8_t  MotorTxDma_GetQueueMaxDepth(MotorId_t motor);

#endif /* MOTOR_TX_DMA_H */
