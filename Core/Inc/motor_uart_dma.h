#ifndef MOTOR_UART_DMA_H
#define MOTOR_UART_DMA_H

#include "app_config.h"

#define MOTOR_DMA_RX_BUFFER_SIZE       128
#define MOTOR_RX_LINE_MAX              128
#define MOTOR_RX_QUEUE_DEPTH           8
#define UART_ERROR_REPORT_INTERVAL_MS  5000

void MotorUartDma_Init(void);
void MotorUartDma_StartAllRx(void);
void MotorUartDma_Update(void);

#endif /* MOTOR_UART_DMA_H */
