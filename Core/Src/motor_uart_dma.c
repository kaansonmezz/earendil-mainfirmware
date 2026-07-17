#include "motor_uart_dma.h"
#include "motor_tx_dma.h"
#include "manipulation_uart_dma.h"
#include "safety_manager.h"
#include "motor_tuning_config.h"
#include "logger.h"
#include <string.h>

#define NUM_MOTOR_UARTS  4

#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

static uint8_t usart2_rx_dma_buffer[MOTOR_DMA_RX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart4_rx_dma_buffer[MOTOR_DMA_RX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart5_rx_dma_buffer[MOTOR_DMA_RX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t uart7_rx_dma_buffer[MOTOR_DMA_RX_BUFFER_SIZE] DMA_BUFFER;

/* ── Per-UART line assembly state ───────────────────────────────────────── */
typedef struct
{
    char            line[MOTOR_RX_LINE_MAX + 1];
    uint16_t        len;
    volatile uint32_t dropped;
} MotorRxLineQueue_t;

static MotorRxLineQueue_t lineQueue[NUM_MOTOR_UARTS];

/* Complete-line SPSC FIFO (ISR producer, main-loop consumer). */
static char     lineBuf[NUM_MOTOR_UARTS][MOTOR_RX_QUEUE_DEPTH][MOTOR_RX_LINE_MAX + 1];
static volatile uint8_t lineHead[NUM_MOTOR_UARTS];
static volatile uint8_t lineTail[NUM_MOTOR_UARTS];
static volatile uint8_t lineCount[NUM_MOTOR_UARTS];

static const char *slotLabel[] = { "USART2_RX", "UART4_RX", "UART5_RX", "UART7_RX" };

/* Motor tag for each RX slot (same ordering as slotLabel / LookupSlot).
 *  USART2=FL, UART4=FR, UART5=RR, UART7=RL                          */
static const char *slotMotorTag[] = { "FL", "FR", "RR", "RL" };

/* ── Per-UART error diagnostic state ────────────────────────────────────── */
typedef struct
{
    UART_HandleTypeDef *huart;
    const char         *name;
    uint8_t            *dmaBuf;

    volatile uint32_t last_error_code;
    volatile uint32_t error_count;

    volatile bool error_active;
    volatile bool immediate_report_pending;
    volatile bool restart_pending;
    volatile bool recovery_pending;

    uint32_t last_error_tick;
    uint32_t last_report_tick;
} MotorUartErrorDiag_t;

static MotorUartErrorDiag_t diag[NUM_MOTOR_UARTS] =
{
    { &huart2, "USART2", usart2_rx_dma_buffer },
    { &huart4, "UART4",  uart4_rx_dma_buffer  },
    { &huart5, "UART5",  uart5_rx_dma_buffer  },
    { &huart7, "UART7",  uart7_rx_dma_buffer  },
};

/* ── Internal: map UART instance → slot index ──────────────────────────── */
static int LookupSlot(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) return 0;
    if (huart->Instance == UART4)  return 1;
    if (huart->Instance == UART5)  return 2;
    if (huart->Instance == UART7)  return 3;
    return -1;
}

/* ── Map UART handle → MotorId_t for safety manager ─────────────────────── */
static bool GetMotorIdFromUart(UART_HandleTypeDef *huart, MotorId_t *motor)
{
    if (huart == NULL || motor == NULL)
        return false;

    if (huart->Instance == USART2) { *motor = MOTOR_FL; return true; }
    if (huart->Instance == UART4)  { *motor = MOTOR_FR; return true; }
    if (huart->Instance == UART7)  { *motor = MOTOR_RL; return true; }
    if (huart->Instance == UART5)  { *motor = MOTOR_RR; return true; }

    return false;
}

/* Pop exactly one complete record.  The copy and metadata update are kept in
 * one short critical section; all parsing and logging happens afterward with
 * interrupts enabled. */
static bool MotorRxQueuePop(uint8_t uartIndex,
                            char out[MOTOR_RX_LINE_MAX + 1])
{
    if (uartIndex >= NUM_MOTOR_UARTS || out == NULL)
        return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (lineCount[uartIndex] == 0U)
    {
        if (!primask) __enable_irq();
        return false;
    }

    uint8_t slot = lineHead[uartIndex];
    memcpy(out, lineBuf[uartIndex][slot], MOTOR_RX_LINE_MAX + 1U);
    lineHead[uartIndex] = (uint8_t)((slot + 1U) % MOTOR_RX_QUEUE_DEPTH);
    lineCount[uartIndex]--;

    if (!primask) __enable_irq();
    return true;
}

/* ── Error report (main-loop context only) ─────────────────────────────── */
static void ReportUartError(MotorUartErrorDiag_t *d, bool isRepeat)
{
    uint32_t error = d->last_error_code;

    Logger_Log(LOG_ERROR, "%s UART error %s: 0x%08lX",
               d->name,
               isRepeat ? "still unresolved" : "code",
               (unsigned long)error);

    if (error & HAL_UART_ERROR_PE)
        Logger_Log(LOG_ERROR, "%s error: PE - Parity error", d->name);

    if (error & HAL_UART_ERROR_NE)
        Logger_Log(LOG_ERROR, "%s error: NE - Noise error", d->name);

    if (error & HAL_UART_ERROR_FE)
        Logger_Log(LOG_ERROR, "%s error: FE - Framing error", d->name);

    if (error & HAL_UART_ERROR_ORE)
        Logger_Log(LOG_ERROR, "%s error: ORE - Overrun error", d->name);

    if (error & HAL_UART_ERROR_DMA)
        Logger_Log(LOG_ERROR, "%s error: DMA - DMA transfer error", d->name);

#ifdef HAL_UART_ERROR_RTO
    if (error & HAL_UART_ERROR_RTO)
        Logger_Log(LOG_ERROR, "%s error: RTO - Receiver timeout error", d->name);
#endif

    d->last_report_tick = HAL_GetTick();
}

/* ── Deferred DMA restart (main-loop context) ──────────────────────────── */
static void ProcessDmaRestart(MotorUartErrorDiag_t *d)
{
    HAL_UART_AbortReceive(d->huart);

    __HAL_UART_CLEAR_FLAG(d->huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                     UART_CLEAR_PEF  | UART_CLEAR_FEF);

    HAL_StatusTypeDef s = HAL_UARTEx_ReceiveToIdle_DMA(
        d->huart, d->dmaBuf, MOTOR_DMA_RX_BUFFER_SIZE);

    if (s != HAL_OK)
    {
        Logger_Log(LOG_ERROR, "%s DMA RX restart failed: %s", d->name,
                   (s == HAL_BUSY)    ? "HAL_BUSY" :
                   (s == HAL_ERROR)   ? "HAL_ERROR" :
                   (s == HAL_TIMEOUT) ? "HAL_TIMEOUT" : "UNKNOWN");
        return;
    }

    if (d->huart->hdmarx != NULL)
        __HAL_DMA_DISABLE_IT(d->huart->hdmarx, DMA_IT_HT);
}

/* ── Start DMA RX on a single UART ─────────────────────────────────────── */
static HAL_StatusTypeDef StartDmaRx(UART_HandleTypeDef *huart, uint8_t *buf, const char *name)
{
    HAL_UART_AbortReceive(huart);

    HAL_StatusTypeDef s = HAL_UARTEx_ReceiveToIdle_DMA(huart, buf, MOTOR_DMA_RX_BUFFER_SIZE);
    if (s != HAL_OK)
    {
        Logger_Log(LOG_ERROR, "%s DMA RX start failed: %d", name, (int)s);
        return s;
    }

    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    else
    {
        Logger_Log(LOG_ERROR, "%s hdmarx is NULL", name);
        return HAL_ERROR;
    }

    Logger_Log(LOG_INFO, "%s DMA RX start OK", name);
    return HAL_OK;
}

/* ── Public functions ────────────────────────────────────────────────────── */

void MotorUartDma_Init(void)
{
    memset(usart2_rx_dma_buffer, 0, sizeof(usart2_rx_dma_buffer));
    memset(uart4_rx_dma_buffer, 0, sizeof(uart4_rx_dma_buffer));
    memset(uart5_rx_dma_buffer, 0, sizeof(uart5_rx_dma_buffer));
    memset(uart7_rx_dma_buffer, 0, sizeof(uart7_rx_dma_buffer));
    memset(lineQueue, 0, sizeof(lineQueue));
    memset((void *)lineHead, 0, sizeof(lineHead));
    memset((void *)lineTail, 0, sizeof(lineTail));
    memset((void *)lineCount, 0, sizeof(lineCount));

    for (int i = 0; i < NUM_MOTOR_UARTS; i++)
    {
        diag[i].last_error_code        = 0;
        diag[i].error_count            = 0;
        diag[i].error_active           = false;
        diag[i].immediate_report_pending = false;
        diag[i].restart_pending        = false;
        diag[i].recovery_pending       = false;
        diag[i].last_error_tick        = 0;
        diag[i].last_report_tick       = 0;
    }

    MotorTuningConfig_Init();
}

void MotorUartDma_StartAllRx(void)
{
    StartDmaRx(&huart2, usart2_rx_dma_buffer, "USART2");
    StartDmaRx(&huart4, uart4_rx_dma_buffer,  "UART4");
    StartDmaRx(&huart5, uart5_rx_dma_buffer,  "UART5");
    StartDmaRx(&huart7, uart7_rx_dma_buffer,  "UART7");
}

void MotorUartDma_Update(void)
{
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < NUM_MOTOR_UARTS; i++)
    {
        MotorUartErrorDiag_t *d = &diag[i];

        /* Deferred DMA restart (after error recovery) */
        if (d->restart_pending)
        {
            d->restart_pending = false;
            ProcessDmaRestart(d);
        }

        /* Immediate error report on first occurrence */
        if (d->immediate_report_pending)
        {
            d->immediate_report_pending = false;
            ReportUartError(d, false);
        }

        /* 5s repeated error report while error remains unresolved */
        if (d->error_active &&
            (now - d->last_report_tick) >= UART_ERROR_REPORT_INTERVAL_MS)
        {
            ReportUartError(d, true);
        }

        /* Recovery notification */
        if (d->recovery_pending)
        {
            d->recovery_pending = false;
            Logger_Log(LOG_INFO, "%s RX recovered after UART error", d->name);
        }
    }

    /* Drain queued complete lines from each UART */
    for (int i = 0; i < NUM_MOTOR_UARTS; i++)
    {
        char line[MOTOR_RX_LINE_MAX + 1];
        while (MotorRxQueuePop((uint8_t)i, line))
        {
            /* Detect compact F411 telemetry.  The payload must START with
             * "RPM:" to be classified as telemetry.  Lines like
             * "[ERR] Unknown commandRPM:0,..." must NOT be classified as
             * telemetry — they are error lines that happen to contain RPM
             * data after the error prefix. */
            if (strncmp(line, "RPM:", 4) == 0 &&
                strstr(line, "PWM_ACT:") != NULL &&
                strstr(line, "RXB:") != NULL)
            {
                Logger_Log(LOG_INFO, "[TEL][%s] %s", slotMotorTag[i], line);
            }
            else
            {
                Logger_Log(LOG_INFO, "[%s] %s", slotLabel[i], line);
            }

            /* Feed every non-telemetry line to the tuning config parser.
             * The parser internally skips telemetry/status lines and only
             * caches Kp_m/Ki_m, Base, and Boost payloads. */
            MotorTuningConfig_ProcessLine(
                MotorTuningConfig_SlotToMotorId(i), line);
        }
    }
}

/* ── HAL weak callback overrides ─────────────────────────────────────────── */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (ManipulationUartDma_HandleRxEvent(huart, Size))
        return;

    int idx = LookupSlot(huart);
    if (idx < 0)
        return;

    if (Size > 0 && Size <= MOTOR_DMA_RX_BUFFER_SIZE)
    {
        MotorRxLineQueue_t *q = &lineQueue[idx];
        const uint8_t *dmaBuf = diag[idx].dmaBuf;

        /* Notify safety manager of motor RX activity for link-loss tracking.
         * Safe to call from ISR: SafetyManager_NotifyRx() only writes a tick
         * value and clears a flag -- no logging, no blocking. */
        MotorId_t motor;
        if (GetMotorIdFromUart(huart, &motor))
        {
            SafetyManager_NotifyRx(motor);
        }

        for (uint16_t j = 0; j < Size; j++)
        {
            uint8_t ch = dmaBuf[j];

            if (ch == '\n')
            {
                q->line[q->len] = '\0';

                if (lineCount[idx] < MOTOR_RX_QUEUE_DEPTH)
                {
                    uint8_t wrSlot = lineTail[idx];
                    memcpy(lineBuf[idx][wrSlot], q->line, q->len + 1);
                    lineTail[idx] = (uint8_t)((wrSlot + 1U) % MOTOR_RX_QUEUE_DEPTH);
                    lineCount[idx]++;
                }
                else
                {
                    q->dropped++;
                }

                q->len = 0;
            }
            else if (ch == '\r')
            {
                /* skip CR */
            }
            else
            {
                if (q->len < MOTOR_RX_LINE_MAX)
                    q->line[q->len++] = (char)ch;
            }
        }
    }

    if (diag[idx].error_active)
    {
        diag[idx].error_active = false;
        diag[idx].last_error_code = 0;
        diag[idx].immediate_report_pending = false;
        diag[idx].recovery_pending = true;
    }

    HAL_StatusTypeDef s = HAL_UARTEx_ReceiveToIdle_DMA(
        huart, diag[idx].dmaBuf, MOTOR_DMA_RX_BUFFER_SIZE);

    if (s == HAL_OK)
    {
        if (huart->hdmarx != NULL)
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    else
    {
        diag[idx].restart_pending = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (ManipulationUartDma_HandleError(huart))
        return;

    int idx = LookupSlot(huart);
    if (idx < 0)
        return;

    uint32_t error = huart->ErrorCode;
    MotorUartErrorDiag_t *d = &diag[idx];

    d->last_error_code = error;
    d->error_count++;
    d->error_active = true;
    d->last_error_tick = HAL_GetTick();

    if (!d->immediate_report_pending)
        d->immediate_report_pending = true;

    d->restart_pending = true;

    /* Route TX DMA errors to motor_tx_dma.c.
     * Only clear TX busy state when a DMA transfer error occurred AND the UART
     * was actively transmitting. Pure RX errors (FE/NE/ORE/PE) must NOT clear
     * a valid TX busy flag — those are handled by the RX recovery path above. */
    bool dmaError = ((error & HAL_UART_ERROR_DMA) != 0U);
    bool txWasActive =
        (huart->gState == HAL_UART_STATE_BUSY_TX) ||
        (huart->gState == HAL_UART_STATE_BUSY_TX_RX);

    if (dmaError && txWasActive)
    {
        MotorTxDma_OnTxError(huart);
    }

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                  UART_CLEAR_PEF  | UART_CLEAR_FEF);
}
