#ifndef MOTOR_TX_DMA_H
#define MOTOR_TX_DMA_H

#include <stdbool.h>
#include "main.h"
#include "rover_types.h"

/* ── Public API ─────────────────────────────────────────────────────────────
 * Motor UART DMA TX module.
 * Owns motor UART DMA TX state and DMA-safe active/pending buffers.
 * All motor UART TX commands are routed through this module.
 * HAL_UART_TxCpltCallback routes TX-complete events to MotorTxDma_OnTxComplete().
 * ─────────────────────────────────────────────────────────────────────────── */

void MotorTxDma_Init(void);

bool MotorTxDma_Send(MotorId_t motor, const char *cmd);
bool MotorTxDma_SendAll(const char *cmd);

void MotorTxDma_OnTxComplete(UART_HandleTypeDef *huart);
void MotorTxDma_OnTxError(UART_HandleTypeDef *huart);

bool MotorTxDma_IsBusy(MotorId_t motor);
bool MotorTxDma_HasPending(MotorId_t motor);
void MotorTxDma_CancelPending(void);  /* drop all queued (non-active) TX frames */

#endif /* MOTOR_TX_DMA_H */
