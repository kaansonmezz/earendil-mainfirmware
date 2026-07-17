#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32h7xx_hal.h"
#include "rover_types.h"

/* ── Terminal (USART3) ──────────────────────────────────────────────────── */
#define TERMINAL_RX_BUF_SIZE    128
#define TERMINAL_TX_BUF_SIZE    256

/* ── Protocol ───────────────────────────────────────────────────────────── */
#define PROTOCOL_FRAME_START    '<'
#define PROTOCOL_FRAME_END      '>'
#define ACK_TIMEOUT_MS          500
#define MAX_RETRIES             3

/* ── Safety ─────────────────────────────────────────────────────────────── */
#define LINK_LOSS_TIMEOUT_MS    3000   /* H7-to-F411 motor UART link loss  */

/* ── PC/Pi control-link watchdog ────────────────────────────────────────── */
#define PC_LINK_HEARTBEAT_PERIOD_MS  500U   /* GUI send interval (info only) */
#define PC_LINK_TIMEOUT_MS          2000U   /* H7 watchdog threshold         */

/* ── Motor UART handle mapping ──────────────────────────────────────────── */
/*  Indexed by MotorId_t: MOTOR_FL=0, MOTOR_FR=1, MOTOR_RL=2, MOTOR_RR=3  */

extern UART_HandleTypeDef huart2;   /* FL */
extern UART_HandleTypeDef huart4;   /* FR */
extern UART_HandleTypeDef huart7;   /* RL */
extern UART_HandleTypeDef huart5;   /* RR */

/*  Helper macro – returns pointer to UART handle for a given MotorId_t     */
#define MOTOR_UART_HANDLE(id)  ( \
    ((id) == MOTOR_FL) ? &huart2 : \
    ((id) == MOTOR_FR) ? &huart4 : \
    ((id) == MOTOR_RL) ? &huart7 : \
    ((id) == MOTOR_RR) ? &huart5 : \
    NULL )

/*  DMA RX handles (used by IRQ handlers in stm32h7xx_it.c)                  */
extern DMA_HandleTypeDef hdma_usart2_rx;   /* FL */
extern DMA_HandleTypeDef hdma_uart4_rx;    /* FR */
extern DMA_HandleTypeDef hdma_uart7_rx;    /* RL */
extern DMA_HandleTypeDef hdma_uart5_rx;    /* RR */

#define MOTOR_DMA_RX_HANDLE(id) ( \
    ((id) == MOTOR_FL) ? &hdma_usart2_rx : \
    ((id) == MOTOR_FR) ? &hdma_uart4_rx : \
    ((id) == MOTOR_RL) ? &hdma_uart7_rx : \
    ((id) == MOTOR_RR) ? &hdma_uart5_rx : \
    NULL )

/* ── Terminal UART handle ───────────────────────────────────────────────── */
extern UART_HandleTypeDef huart3;

/* ── Manipulation UART8 handle ───────────────────────────────────────────── */
extern UART_HandleTypeDef huart8;
extern DMA_HandleTypeDef  hdma_uart8_rx;
extern DMA_HandleTypeDef  hdma_uart8_tx;

#endif /* APP_CONFIG_H */
