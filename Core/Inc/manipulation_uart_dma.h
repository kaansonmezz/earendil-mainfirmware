#ifndef MANIPULATION_UART_DMA_H
#define MANIPULATION_UART_DMA_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

void ManipulationUartDma_Init(void);
void ManipulationUartDma_StartRx(void);
void ManipulationUartDma_Update(void);

bool ManipulationUartDma_SendRaw(const char *payload);

bool ManipulationUartDma_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t size);
bool ManipulationUartDma_HandleError(UART_HandleTypeDef *huart);
void ManipulationUartDma_OnTxComplete(UART_HandleTypeDef *huart);

#endif
