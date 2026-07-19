#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32h7xx_hal.h"
#include "rover_types.h"

/* ── Shared I2C1 timing constant ──────────────────────────────────────────
 * I2C1 kernel clock = 130 MHz (RCC.I2C123Freq_Value, APB1 default source).
 *
 * TIMINGR field decode:
 *   [31:28] PRESC   = 0x4 = 4   → tPRESC = 130 MHz / (4+1) = 26 MHz → 38.46 ns
 *   [27:24] reserved = 0x0      → must be zero
 *   [23:20] SCLDEL  = 0x3 = 3   → data setup time = 3 × 38.46 ns = 115 ns
 *   [19:16] SDADEL  = 0x2 = 2   → data hold time  = 2 × 38.46 ns = 77 ns
 *   [15:8]  SCLH    = 0xF9 = 249 → SCL high period = 250 × 38.46 ns = 9.615 µs
 *   [7:0]   SCLL    = 0xF9 = 249 → SCL low period  = 250 × 38.46 ns = 9.615 µs
 *   SCL period = (SCLH+1 + SCLL+1) × tPRESC = 500 × 38.46 ns = 19.23 µs
 *   SCL frequency ≈ 52 kHz
 *
 * Original CubeMX value 0x20A0ACFE gave ~100 kHz.
 * Final measurement must be confirmed with oscilloscope or logic analyzer on PB8/PB9. */
#define I2C_TIMING_APP  0x4032F9F9U

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
