#include "manipulation_uart_dma.h"
#include "app_config.h"
#include "logger.h"
#include <string.h>

/* ── Configuration ──	g──────────────────────────────────────────────────────── */
#define MANIP_DMA_RX_BUFFER_SIZE           128U
#define MANIP_TX_FRAME_MAX                 96U
#define MANIP_TX_QUEUE_DEPTH               8U
#define MANIP_RX_LINE_MAX                  160U
#define MANIP_RX_QUEUE_DEPTH               8U
#define MANIP_UART_ERROR_REPORT_INTERVAL_MS 5000U

#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

/* ── TX frame queue entry ─────────────────────────────────────────────────── */
typedef struct
{
    uint8_t  data[MANIP_TX_FRAME_MAX];
    uint16_t len;
} ManipTxFrame_t;

/* ── DMA-safe buffers ─────────────────────────────────────────────────────── */
static uint8_t s_rx_dma_buf[MANIP_DMA_RX_BUFFER_SIZE] DMA_BUFFER;
static uint8_t s_tx_active_buf[MANIP_TX_FRAME_MAX]    DMA_BUFFER;

/* ── TX queue state ───────────────────────────────────────────────────────── */
static ManipTxFrame_t  s_tx_queue[MANIP_TX_QUEUE_DEPTH];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile uint8_t s_tx_count;
static volatile uint32_t s_tx_dropped;
static volatile bool     s_tx_busy;

/* Deferred TX log (avoid logging from ISR context) */
static char     s_tx_log_payload[MANIP_TX_FRAME_MAX + 1];
static volatile bool s_tx_log_pending;

/* Deferred TX log counters (avoid logging from ISR context) */
static volatile uint32_t s_tx_hal_failures;
static volatile uint32_t s_tx_queue_full_count;
static volatile uint32_t s_tx_payload_too_long_count;

/* ── RX line assembly state (written in ISR context) ──────────────────────── */
static char     s_assembly_line[MANIP_RX_LINE_MAX + 1];
static uint16_t s_assembly_len;
static bool     s_assembly_overflow;

/* Complete-line SPSC FIFO (ISR producer, main-loop consumer). */
static char     s_line_queue[MANIP_RX_QUEUE_DEPTH][MANIP_RX_LINE_MAX + 1];
static volatile uint8_t s_line_head;
static volatile uint8_t s_line_tail;
static volatile uint8_t s_line_count;
static volatile uint32_t s_dropped_rx_lines;
static volatile uint32_t s_rx_line_overflow_pending;
static volatile uint32_t s_rx_queue_full_pending;

/* ── UART error diagnostic state ──────────────────────────────────────────── */
static volatile uint32_t s_last_error_code;
static volatile uint32_t s_error_count;
static volatile bool     s_error_active;
static volatile bool     s_immediate_report_pending;
static volatile bool     s_restart_pending;
static volatile bool     s_recovery_pending;
static uint32_t          s_last_error_tick;
static uint32_t          s_last_report_tick;

/* ── Internal state ───────────────────────────────────────────────────────── */
static bool     s_initialized;
static bool     s_rx_started;

/* ── Internal helpers ─────────────────────────────────────────────────────── */
static bool IsManipulationUart(UART_HandleTypeDef *huart)
{
    return (huart != NULL) && (huart->Instance == UART8);
}

/* Full RX DMA start (clean/recovery path).
 * Aborts any pending receive, clears error flags, arms ReceiveToIdle DMA,
 * and disables the half-transfer interrupt.  Used by StartRx() and deferred
 * recovery in Update(). */
static HAL_StatusTypeDef ArmRxDma(void)
{
    HAL_UART_AbortReceive(&huart8);

    __HAL_UART_CLEAR_FLAG(&huart8, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                    UART_CLEAR_PEF  | UART_CLEAR_FEF);

    HAL_StatusTypeDef s = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart8, s_rx_dma_buf, MANIP_DMA_RX_BUFFER_SIZE);

    if (s != HAL_OK)
        return s;

    if (huart8.hdmarx != NULL)
        __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);

    return HAL_OK;
}

/* Lightweight RX DMA re-arm (used inside HandleRxEvent only).
 * Does NOT call HAL_UART_AbortReceive — the receive is already idle after
 * the ReceiveToIdle callback fired.  Only re-arms the DMA and disables
 * the half-transfer interrupt. */
static HAL_StatusTypeDef ReArmRxDma(void)
{
    HAL_StatusTypeDef s = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart8, s_rx_dma_buf, MANIP_DMA_RX_BUFFER_SIZE);

    if (s != HAL_OK)
        return s;

    if (huart8.hdmarx != NULL)
        __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);

    return HAL_OK;
}

/* ── Error report (main-loop context only) ────────────────────────────────── */
static void ReportManipUartError(bool isRepeat)
{
    uint32_t error = s_last_error_code;

    Logger_Log(LOG_ERROR, "UART8 UART error %s: 0x%08lX",
               isRepeat ? "still unresolved" : "code",
               (unsigned long)error);

    if (error & HAL_UART_ERROR_PE)
        Logger_Log(LOG_ERROR, "UART8 error: PE - Parity error");

    if (error & HAL_UART_ERROR_NE)
        Logger_Log(LOG_ERROR, "UART8 error: NE - Noise error");

    if (error & HAL_UART_ERROR_FE)
        Logger_Log(LOG_ERROR, "UART8 error: FE - Framing error");

    if (error & HAL_UART_ERROR_ORE)
        Logger_Log(LOG_ERROR, "UART8 error: ORE - Overrun error");

    if (error & HAL_UART_ERROR_DMA)
        Logger_Log(LOG_ERROR, "UART8 error: DMA - DMA transfer error");

#ifdef HAL_UART_ERROR_RTO
    if (error & HAL_UART_ERROR_RTO)
        Logger_Log(LOG_ERROR, "UART8 error: RTO - Receiver timeout error");
#endif

    s_last_report_tick = HAL_GetTick();
}

/* ── TX queue helpers ───────────────────────────────────────────────────────
 * The queue is accessed from both main-loop context (SendRaw) and
 * ISR/callback context (OnTxComplete).  Briefly disable IRQs around the
 * index/count updates to keep state consistent. */

static bool TxQueueIsFull(void)
{
    return s_tx_count >= MANIP_TX_QUEUE_DEPTH;
}

static bool TxQueueIsEmpty(void)
{
    return s_tx_count == 0U;
}

static bool TxQueuePush(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > MANIP_TX_FRAME_MAX)
        return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (TxQueueIsFull())
    {
        s_tx_dropped++;
        s_tx_queue_full_count++;
        if (!primask) __enable_irq();
        return false;
    }

    ManipTxFrame_t *slot = &s_tx_queue[s_tx_tail];
    memcpy(slot->data, data, len);
    slot->len = len;

    s_tx_tail = (uint8_t)((s_tx_tail + 1U) % MANIP_TX_QUEUE_DEPTH);
    s_tx_count++;

    if (!primask) __enable_irq();
    return true;
}

/* Pop a frame.  Caller must already hold a critical section. */
static bool TxQueuePopLocked(ManipTxFrame_t *out)
{
    if (out == NULL || TxQueueIsEmpty())
        return false;

    *out = s_tx_queue[s_tx_head];
    s_tx_head = (uint8_t)((s_tx_head + 1U) % MANIP_TX_QUEUE_DEPTH);
    s_tx_count--;
    return true;
}

/* Pop one RX record atomically.  Logger_Log() is intentionally called only
 * after this function returns and interrupts have been restored. */
static bool RxQueuePop(char out[MANIP_RX_LINE_MAX + 1])
{
    if (out == NULL)
        return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_line_count == 0U)
    {
        if (!primask) __enable_irq();
        return false;
    }

    uint8_t slot = s_line_head;
    memcpy(out, s_line_queue[slot], MANIP_RX_LINE_MAX + 1U);
    s_line_head = (uint8_t)((slot + 1U) % MANIP_RX_QUEUE_DEPTH);
    s_line_count--;

    if (!primask) __enable_irq();
    return true;
}

/* Snapshot and clear pending drop categories without losing an ISR increment
 * between the snapshot and reset.  The lifetime total is never reset. */
static void TakeRxDropCounters(uint32_t *lineOverflow,
                               uint32_t *queueFull,
                               uint32_t *totalDropped)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    *lineOverflow = s_rx_line_overflow_pending;
    *queueFull = s_rx_queue_full_pending;
    *totalDropped = s_dropped_rx_lines;
    s_rx_line_overflow_pending = 0U;
    s_rx_queue_full_pending = 0U;

    if (!primask) __enable_irq();
}

/* ── TX DMA start logic ─────────────────────────────────────────────────────
 * Pops the next queued frame, copies it into the active DMA buffer, and
 * starts HAL_UART_Transmit_DMA().
 *
 * Busy-check + pop + set-busy is atomic under a short critical section.
 * The HAL call is done AFTER re-enabling IRQs.
 *
 * No Logger_Log() calls here — this function may be called from the TX
 * complete ISR callback.  TX logging is deferred to Update(). */
static void ManipTx_TryStartNext(void)
{
    ManipTxFrame_t frame;
    bool popped;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_tx_busy)
    {
        if (!primask) __enable_irq();
        return;
    }

    popped = TxQueuePopLocked(&frame);
    if (!popped)
    {
        if (!primask) __enable_irq();
        return;
    }

    /* Reserve TX BEFORE releasing the lock so that a racing TxCplt
     * callback sees busy==true and does not double-start. */
    s_tx_busy = true;

    if (!primask) __enable_irq();

    /* Copy into the dedicated active DMA buffer.  DMA needs stable memory
     * until the TxCplt callback fires; the queue slot may be reused
     * immediately after we pop. */
    memcpy(s_tx_active_buf, frame.data, frame.len);

    if (HAL_UART_Transmit_DMA(&huart8, s_tx_active_buf, frame.len) != HAL_OK)
    {
        /* HAL failed to start.  Mark not-busy, count as dropped. */
        primask = __get_PRIMASK();
        __disable_irq();
        s_tx_busy = false;
        s_tx_dropped++;
        s_tx_hal_failures++;
        if (!primask) __enable_irq();

        /* Attempt the next queued frame.  Recursion depth is bounded by
         * the finite queue (MANIP_TX_QUEUE_DEPTH). */
        ManipTx_TryStartNext();
        return;
    }

    /* Defer [ARM_TX] logging to Update().  Copy only the payload portion
     * (exclude the trailing \r\n).  The frame always ends with \r\n when
     * built by SendRaw, so frame.len >= 2 is guaranteed. */
    uint16_t payloadLen = frame.len;
    if (payloadLen >= 2U)
        payloadLen -= 2U;

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(s_tx_log_payload, frame.data, payloadLen);
    s_tx_log_payload[payloadLen] = '\0';
    s_tx_log_pending = true;
    if (!primask) __enable_irq();
}

/* ── Public functions ─────────────────────────────────────────────────────── */

void ManipulationUartDma_Init(void)
{
    s_initialized = false;
    s_rx_started  = false;
    s_tx_busy     = false;

    s_tx_head   = 0U;
    s_tx_tail   = 0U;
    s_tx_count  = 0U;
    s_tx_dropped = 0U;
    s_tx_hal_failures = 0U;
    s_tx_queue_full_count = 0U;
    s_tx_payload_too_long_count = 0U;

    s_tx_log_pending = false;
    memset(s_tx_log_payload, 0, sizeof(s_tx_log_payload));

    s_dropped_rx_lines = 0U;
    s_rx_line_overflow_pending = 0U;
    s_rx_queue_full_pending = 0U;

    s_last_error_code    = 0U;
    s_error_count        = 0U;
    s_error_active       = false;
    s_immediate_report_pending = false;
    s_restart_pending    = false;
    s_recovery_pending   = false;
    s_last_error_tick    = 0U;
    s_last_report_tick   = 0U;

    s_assembly_len = 0U;
    s_assembly_overflow = false;
    s_line_head    = 0U;
    s_line_tail    = 0U;
    s_line_count   = 0U;

    memset(s_rx_dma_buf, 0, sizeof(s_rx_dma_buf));
    memset(s_tx_active_buf, 0, sizeof(s_tx_active_buf));
    memset(s_assembly_line, 0, sizeof(s_assembly_line));
    memset(s_line_queue, 0, sizeof(s_line_queue));
    memset(s_tx_queue, 0, sizeof(s_tx_queue));

    s_initialized = true;

    Logger_Log(LOG_INFO, "[MANIP] UART8 manipulation channel initialized");
}

void ManipulationUartDma_StartRx(void)
{
    if (!s_initialized)
        return;

    if (ArmRxDma() != HAL_OK)
    {
        Logger_Log(LOG_ERROR, "[MANIP] UART8 DMA RX start failed");
        s_restart_pending = true;
        return;
    }

    s_rx_started = true;
    Logger_Log(LOG_INFO, "[MANIP] UART8 DMA RX start OK");
}

void ManipulationUartDma_Update(void)
{
    if (!s_initialized)
        return;

    uint32_t now = HAL_GetTick();

    /* ── Deferred DMA restart ─────────────────────────────────────── */
    if (s_restart_pending)
    {
        s_restart_pending = false;
        if (ArmRxDma() == HAL_OK)
        {
            if (!s_rx_started)
                s_rx_started = true;
            Logger_Log(LOG_INFO, "[MANIP] UART8 DMA RX restart OK");
        }
        else
        {
            s_restart_pending = true;
        }
    }

    /* ── Immediate error report on first occurrence ───────────────── */
    if (s_immediate_report_pending)
    {
        s_immediate_report_pending = false;
        ReportManipUartError(false);
    }

    /* ── 5s repeated error report while error remains unresolved ──── */
    if (s_error_active &&
        (now - s_last_report_tick) >= MANIP_UART_ERROR_REPORT_INTERVAL_MS)
    {
        ReportManipUartError(true);
    }

    /* ── Recovery notification ────────────────────────────────────── */
    if (s_recovery_pending)
    {
        s_recovery_pending = false;
        Logger_Log(LOG_INFO, "UART8 RX recovered after UART error");
    }

    /* ── Deferred TX log ──────────────────────────────────────────── */
    if (s_tx_log_pending)
    {
        s_tx_log_pending = false;
        Logger_Log(LOG_INFO, "[ARM_TX] %s", s_tx_log_payload);
    }

    /* ── Drain queued complete lines ──────────────────────────────── */
    char rxLine[MANIP_RX_LINE_MAX + 1];
    while (RxQueuePop(rxLine))
    {
        Logger_Log(LOG_INFO, "[ARM_RX] %s", rxLine);
    }

    /* ── RX overflow warning ──────────────────────────────────────── */
    uint32_t lineOverflow;
    uint32_t queueFull;
    uint32_t totalDropped;
    TakeRxDropCounters(&lineOverflow, &queueFull, &totalDropped);
    if (lineOverflow > 0U || queueFull > 0U)
    {
        Logger_Log(LOG_WARN,
                   "[MANIP] UART8 RX drops: line_overflow=%lu queue_full=%lu total=%lu",
                   (unsigned long)lineOverflow,
                   (unsigned long)queueFull,
                   (unsigned long)totalDropped);
    }

    /* ── Deferred TX warning aggregation ──────────────────────────── */
    uint32_t halFails = s_tx_hal_failures;
    uint32_t qFull    = s_tx_queue_full_count;
    uint32_t tooLong  = s_tx_payload_too_long_count;

    if (halFails > 0U || qFull > 0U || tooLong > 0U)
    {
        Logger_Log(LOG_WARN,
                   "[MANIP] UART8 TX warnings: hal_fail=%lu queue_full=%lu payload_too_long=%lu total_dropped=%lu",
                   (unsigned long)halFails,
                   (unsigned long)qFull,
                   (unsigned long)tooLong,
                   (unsigned long)s_tx_dropped);
        s_tx_hal_failures        = 0U;
        s_tx_queue_full_count    = 0U;
        s_tx_payload_too_long_count = 0U;
    }
}

bool ManipulationUartDma_SendRaw(const char *payload)
{
    if (!s_initialized || payload == NULL)
        return false;

    /* Measure payload length */
    uint16_t payloadLen = 0U;
    while (payload[payloadLen] != '\0')
    {
        payloadLen++;
        if (payloadLen >= MANIP_TX_FRAME_MAX)
        {
            s_tx_dropped++;
            s_tx_payload_too_long_count++;
            return false;
        }
    }
    if (payloadLen == 0U)
        return false;

    /* Build frame with \r\n suffix into a stack buffer.
     * Total frame length = payloadLen + 2 (\r\n). */
    uint16_t frameLen = payloadLen + 2U;
    if (frameLen > MANIP_TX_FRAME_MAX)
    {
        s_tx_dropped++;
        s_tx_payload_too_long_count++;
        return false;
    }

    uint8_t frameBuf[MANIP_TX_FRAME_MAX];
    memcpy(frameBuf, payload, payloadLen);
    frameBuf[payloadLen]     = '\r';
    frameBuf[payloadLen + 1] = '\n';

    /* Push into the TX queue */
    if (!TxQueuePush(frameBuf, frameLen))
    {
        /* Queue full — drop counted inside TxQueuePush */
        return false;
    }

    /* Kick the TX DMA state machine.  If idle, the just-pushed frame
     * starts immediately.  If busy, TryStartNext returns early and
     * the TxCplt callback will drain the queue when the active frame
     * ends. */
    ManipTx_TryStartNext();

    return true;
}

bool ManipulationUartDma_HandleRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (!IsManipulationUart(huart))
        return false;

    /* Clamp size to DMA buffer bounds */
    if (size > MANIP_DMA_RX_BUFFER_SIZE)
        size = MANIP_DMA_RX_BUFFER_SIZE;

    /* Assemble lines from the received DMA chunk */
    for (uint16_t i = 0U; i < size; i++)
    {
        uint8_t ch = s_rx_dma_buf[i];

        if (ch == '\n')
        {
            if (s_assembly_overflow)
            {
                s_dropped_rx_lines++;
                s_rx_line_overflow_pending++;
            }
            else
            {
                s_assembly_line[s_assembly_len] = '\0';

                if (s_line_count < MANIP_RX_QUEUE_DEPTH)
                {
                    uint8_t wrSlot = s_line_tail;
                    memcpy(s_line_queue[wrSlot], s_assembly_line,
                           s_assembly_len + 1U);
                    s_line_tail = (uint8_t)((wrSlot + 1U) % MANIP_RX_QUEUE_DEPTH);
                    s_line_count++;
                }
                else
                {
                    s_dropped_rx_lines++;
                    s_rx_queue_full_pending++;
                }
            }

            s_assembly_len = 0U;
            s_assembly_overflow = false;
        }
        else if (ch == '\r')
        {
            /* skip CR */
        }
        else
        {
            if (s_assembly_overflow)
            {
                /* Discard the remainder of this overlong record. */
            }
            else if (s_assembly_len < MANIP_RX_LINE_MAX)
                s_assembly_line[s_assembly_len++] = (char)ch;
            else
                s_assembly_overflow = true;
        }
    }

    /* Recovery notification: was in error state, now got valid RX.
     * Deferred to Update() so the log is emitted from main-loop context. */
    if (s_error_active)
    {
        s_error_active    = false;
        s_last_error_code = 0U;
        s_immediate_report_pending = false;
        s_recovery_pending = true;
    }

    /* Lightweight re-arm (no AbortReceive — we are already inside the
     * ReceiveToIdle callback so the receiver is idle). */
    if (ReArmRxDma() != HAL_OK)
        s_restart_pending = true;

    return true;
}

bool ManipulationUartDma_HandleError(UART_HandleTypeDef *huart)
{
    if (!IsManipulationUart(huart))
        return false;

    uint32_t error = huart->ErrorCode;

    s_last_error_code = error;
    s_error_count++;
    s_error_active    = true;
    s_last_error_tick = HAL_GetTick();

    if (!s_immediate_report_pending)
        s_immediate_report_pending = true;

    /* Schedule DMA restart */
    s_restart_pending = true;

    /* Clear UART error flags */
    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                  UART_CLEAR_PEF  | UART_CLEAR_FEF);

    /* TX error cleanup: if a DMA error occurred while UART8 was actively
     * transmitting, clear the TX busy flag so queued frames are not stuck.
     * RX-only errors (FE/NE/ORE/PE) must not clear a valid TX busy state. */
    bool dmaError = ((error & HAL_UART_ERROR_DMA) != 0U);
    bool txWasActive =
        (huart->gState == HAL_UART_STATE_BUSY_TX) ||
        (huart->gState == HAL_UART_STATE_BUSY_TX_RX);

    if (dmaError && txWasActive)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        s_tx_busy = false;
        if (!primask) __enable_irq();
    }

    return true;
}

void ManipulationUartDma_OnTxComplete(UART_HandleTypeDef *huart)
{
    if (!IsManipulationUart(huart))
        return;

    /* Clear busy (the active DMA transfer just completed).
     * No Logger_Log() here — this runs in DMA ISR context. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_tx_busy = false;
    if (!primask) __enable_irq();

    ManipTx_TryStartNext();
}
