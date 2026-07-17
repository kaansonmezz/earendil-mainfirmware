#include "motor_tx_dma.h"
#include "manipulation_uart_dma.h"
#include "app_config.h"
#include "logger.h"
#include <string.h>

/* ── Configuration ────────────────────────────────────────────────────────── */
#define MOTOR_TX_DMA_BUFFER_SIZE 64U
#define MOTOR_TX_QUEUE_DEPTH     16U
#define MOTOR_TX_DMA_COUNT       MOTOR_COUNT

/* DMA-safe section attribute – matches the style used for RX DMA buffers. */
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

/* ── Per-frame queue entry ────────────────────────────────────────────────── */
typedef struct
{
    uint8_t  data[MOTOR_TX_DMA_BUFFER_SIZE];
    uint16_t len;
    bool     safety;       /* true if stop/brake */
} MotorTxFrame_t;

/* ── Per-motor channel state ──────────────────────────────────────────────── */
typedef struct
{
    MotorId_t           motor;
    UART_HandleTypeDef *huart;

    uint8_t            *txBuffer;     /* active DMA buffer (stable until TxCplt) */
    uint16_t            txLen;
    bool                busy;

    MotorTxFrame_t      queue[MOTOR_TX_QUEUE_DEPTH];
    volatile uint8_t    head;         /* pop index */
    volatile uint8_t    tail;         /* push index */
    volatile uint8_t    count;

    uint32_t            dropped;      /* queue-full or HAL-fail counter */
    uint8_t             maxDepth;     /* high-watermark for debug */
} MotorTxDmaChannel_t;

/* ── DMA-safe active TX buffers (one per motor) ───────────────────────────── */
static uint8_t fl_tx_buffer[MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t fr_tx_buffer[MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t rl_tx_buffer[MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t rr_tx_buffer[MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;

/* ── Channel table ──────────────────────────────────────────────────────────
 * Uses the same motor-to-UART mapping as motor_dispatcher.c / app_config.h
 *   FL -> huart2 (USART2)
 *   FR -> huart4 (UART4)
 *   RL -> huart7 (UART7)
 *   RR -> huart5 (UART5)
 * ─────────────────────────────────────────────────────────────────────────── */
static MotorTxDmaChannel_t s_channels[MOTOR_TX_DMA_COUNT] =
{
    { .motor = MOTOR_FL, .huart = &huart2, .txBuffer = fl_tx_buffer },
    { .motor = MOTOR_FR, .huart = &huart4, .txBuffer = fr_tx_buffer },
    { .motor = MOTOR_RL, .huart = &huart7, .txBuffer = rl_tx_buffer },
    { .motor = MOTOR_RR, .huart = &huart5, .txBuffer = rr_tx_buffer },
};

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static MotorTxDmaChannel_t *FindChannelByUart(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < MOTOR_TX_DMA_COUNT; i++)
    {
        if (s_channels[i].huart == huart)
            return &s_channels[i];
    }
    return NULL;
}

/* Returns true if cmd is a safety-critical payload ("stop", "x", or "brake"),
 * tolerating optional trailing CR/LF. No heavy parsing — simple length +
 * char compare so it is safe to call from any context. */
static bool IsSafetyCommand(const char *cmd, uint16_t len)
{
    uint16_t end = len;
    while (end > 0U && (cmd[end - 1U] == '\r' || cmd[end - 1U] == '\n'))
        end--;

    if (end == 4U &&
        cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'o' && cmd[3] == 'p')
        return true;

    if (end == 5U &&
        cmd[0] == 'b' && cmd[1] == 'r' && cmd[2] == 'a' &&
        cmd[3] == 'k' && cmd[4] == 'e')
        return true;

    if (end == 1U && cmd[0] == 'x')
        return true;

    return false;
}

/* ── Queue helpers ──────────────────────────────────────────────────────────
 * The queue is accessed from both main-loop context (MotorTxDma_Send) and
 * ISR/callback context (MotorTxDma_OnTxComplete).  Briefly disable IRQs
 * around the index/count updates to keep state consistent. */
static bool QueueIsFull(const MotorTxDmaChannel_t *ch)
{
    return ch->count >= MOTOR_TX_QUEUE_DEPTH;
}

static bool QueueIsEmpty(const MotorTxDmaChannel_t *ch)
{
    return ch->count == 0U;
}

static void QueueClearLocked(MotorTxDmaChannel_t *ch)
{
    ch->head  = 0U;
    ch->tail  = 0U;
    ch->count = 0U;
}

static bool QueuePush(MotorTxDmaChannel_t *ch,
                      const uint8_t *data,
                      uint16_t len,
                      bool safety)
{
    if (data == NULL || len == 0U || len >= MOTOR_TX_DMA_BUFFER_SIZE)
        return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    bool full = QueueIsFull(ch);
    if (full)
    {
        ch->dropped++;
        if (!primask) __enable_irq();
        return false;
    }

    MotorTxFrame_t *slot = &ch->queue[ch->tail];
    memcpy(slot->data, data, len);
    slot->len     = len;
    slot->safety  = safety;

    ch->tail = (uint8_t)((ch->tail + 1U) % MOTOR_TX_QUEUE_DEPTH);
    ch->count++;

    if (ch->count > ch->maxDepth)
        ch->maxDepth = ch->count;

    if (!primask) __enable_irq();
    return true;
}

/* Pop a frame without acquiring the IRQ lock — the caller must already
 * hold a critical section.  Used by TryStartNext which needs the busy-check
 * and pop to be atomic together. */
static bool QueuePopLocked(MotorTxDmaChannel_t *ch, MotorTxFrame_t *out)
{
    if (out == NULL || QueueIsEmpty(ch))
        return false;

    *out = ch->queue[ch->head];
    ch->head = (uint8_t)((ch->head + 1U) % MOTOR_TX_QUEUE_DEPTH);
    ch->count--;
    return true;
}

static void QueueClear(MotorTxDmaChannel_t *ch)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    QueueClearLocked(ch);
    if (!primask) __enable_irq();
}

/* ── DMA transmit start logic ───────────────────────────────────────────────
 * Pops the next queued frame, copies it into the channel's stable active
 * DMA buffer, and starts HAL_UART_Transmit_DMA().
 *
 * The busy-check + pop + set-busy sequence is performed atomically under a
 * short critical section so that two callers (main-loop Send vs ISR TxCplt)
 * cannot both pop a frame.  The HAL call is done AFTER re-enabling IRQs to
 * avoid holding interrupts disabled while the HAL state machine runs. */
static void MotorTxDma_TryStartNext(MotorTxDmaChannel_t *ch)
{
    if (ch == NULL)
        return;

    MotorTxFrame_t frame;
    bool popped;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (ch->busy)
    {
        if (!primask) __enable_irq();
        return;
    }

    popped = QueuePopLocked(ch, &frame);
    if (!popped)
    {
        if (!primask) __enable_irq();
        return;
    }

    /* Reserve the channel BEFORE releasing the lock so that a racing
     * TxCplt callback sees busy==true and does not double-start. */
    ch->busy = true;

    if (!primask) __enable_irq();

    /* Copy into the channel's dedicated active DMA buffer.  DMA needs stable
     * memory until the TxCplt callback fires; the queue slot may be reused
     * for the next push immediately after we pop. */
    memcpy(ch->txBuffer, frame.data, frame.len);
    ch->txLen = frame.len;

    if (HAL_UART_Transmit_DMA(ch->huart, ch->txBuffer, ch->txLen) != HAL_OK)
    {
        /* HAL failed to start.  Mark not-busy, count as dropped. */
        primask = __get_PRIMASK();
        __disable_irq();
        ch->busy = false;
        ch->dropped++;
        if (!primask) __enable_irq();

        /* Attempt the next queued frame.  Recursion depth is bounded by
         * the finite queue (MOTOR_TX_QUEUE_DEPTH). */
        MotorTxDma_TryStartNext(ch);
        return;
    }
}

/* ── Public functions ─────────────────────────────────────────────────────── */

void MotorTxDma_Init(void)
{
    for (int i = 0; i < MOTOR_TX_DMA_COUNT; i++)
    {
        MotorTxDmaChannel_t *ch = &s_channels[i];

        ch->motor     = (MotorId_t)i;
        ch->huart     = MOTOR_UART_HANDLE((MotorId_t)i);
        ch->txLen     = 0;
        ch->busy      = false;
        ch->head      = 0;
        ch->tail      = 0;
        ch->count     = 0;
        ch->dropped   = 0;
        ch->maxDepth  = 0;

        memset(ch->txBuffer, 0, MOTOR_TX_DMA_BUFFER_SIZE);
        memset(ch->queue,    0, sizeof(ch->queue));
    }
}

bool MotorTxDma_Send(MotorId_t motor, const char *cmd)
{
    if (motor >= MOTOR_COUNT || cmd == NULL)
        return false;

    uint16_t len = 0;
    while (cmd[len] != '\0')
    {
        len++;
        if (len >= MOTOR_TX_DMA_BUFFER_SIZE)
            return false;
    }
    if (len == 0)
        return false;

    MotorTxDmaChannel_t *ch = &s_channels[motor];

    bool newIsSafety = IsSafetyCommand(cmd, len);

    /* Safety commands flush queued normal commands so the stop/brake is
     * transmitted immediately next, after the active frame finishes. */
    if (newIsSafety)
    {
        QueueClear(ch);
        Logger_Log(LOG_INFO, "[MOTOR-TX] %s safety command, queue cleared",
                   (motor == MOTOR_FL) ? "FL" :
                   (motor == MOTOR_FR) ? "FR" :
                   (motor == MOTOR_RL) ? "RL" : "RR");
    }

    bool ok = QueuePush(ch, (const uint8_t *)cmd, len, newIsSafety);
    if (!ok)
    {
        Logger_Log(LOG_WARN, "[MOTOR-TX-WARN] %s queue full, dropped=%lu",
                   (motor == MOTOR_FL) ? "FL" :
                   (motor == MOTOR_FR) ? "FR" :
                   (motor == MOTOR_RL) ? "RL" : "RR",
                   (unsigned long)ch->dropped);
        return false;
    }

    Logger_Log(LOG_DEBUG, "[MOTOR-TX] %s queued len=%u count=%u",
               (motor == MOTOR_FL) ? "FL" :
               (motor == MOTOR_FR) ? "FR" :
               (motor == MOTOR_RL) ? "RL" : "RR",
               (unsigned)len, (unsigned)ch->count);

    /* Kick the DMA state machine.  If the channel is idle, the just-pushed
     * frame starts immediately.  If busy, TryStartNext returns early and
     * the TxCplt callback will drain the queue when the active frame ends. */
    MotorTxDma_TryStartNext(ch);

    return true;
}

bool MotorTxDma_SendAll(const char *cmd)
{
    bool ok = true;

    for (int i = 0; i < MOTOR_TX_DMA_COUNT; i++)
    {
        if (!MotorTxDma_Send((MotorId_t)i, cmd))
            ok = false;
    }
    return ok;
}

void MotorTxDma_OnTxComplete(UART_HandleTypeDef *huart)
{
    MotorTxDmaChannel_t *ch = FindChannelByUart(huart);
    if (ch == NULL)
        return;

    /* Clear busy (the active DMA transfer just completed).
     * NOTE: no logging here — this callback runs in DMA ISR context where
     * the logger's blocking HAL_UART_Transmit(&huart3) would deadlock. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ch->busy = false;
    if (!primask) __enable_irq();

    MotorTxDma_TryStartNext(ch);
}

void MotorTxDma_OnTxError(UART_HandleTypeDef *huart)
{
    MotorTxDmaChannel_t *ch = FindChannelByUart(huart);
    if (ch == NULL)
        return;

    /* Clear busy on TX error.  Queued frames are PRESERVED so that a
     * safety-critical stop/brake queued before the error is not lost.
     * The next successful MotorTxDma_Send() on an idle channel or the
     * next TryStartNext will flush it. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    ch->busy = false;
    if (!primask) __enable_irq();
}

bool MotorTxDma_IsBusy(MotorId_t motor)
{
    if (motor >= MOTOR_COUNT)
        return false;
    return s_channels[motor].busy;
}

bool MotorTxDma_HasPending(MotorId_t motor)
{
    if (motor >= MOTOR_COUNT)
        return false;
    return s_channels[motor].count > 0U;
}

bool MotorTxDma_AllIdle(void)
{
    for (int i = 0; i < MOTOR_TX_DMA_COUNT; i++)
    {
        if (s_channels[i].busy || s_channels[i].count > 0U)
            return false;
    }
    return true;
}

void MotorTxDma_CancelPending(void)
{
    /* Drop every queued (not-yet-started) TX frame on all motor channels.
     * Active DMA transfers are left alone (they will complete and raise
     * TxCplt).  This guarantees a motion frame staged before DISARM cannot
     * fire after the lock is released — defense against stale commands. */
    for (int i = 0; i < MOTOR_TX_DMA_COUNT; i++)
    {
        QueueClear(&s_channels[i]);
    }
}

/* ── Optional debug getters ────────────────────────────────────────────────── */
uint8_t MotorTxDma_GetQueueCount(MotorId_t motor)
{
    if (motor >= MOTOR_COUNT)
        return 0;
    return s_channels[motor].count;
}

uint32_t MotorTxDma_GetDroppedCount(MotorId_t motor)
{
    if (motor >= MOTOR_COUNT)
        return 0;
    return s_channels[motor].dropped;
}

uint8_t MotorTxDma_GetQueueMaxDepth(MotorId_t motor)
{
    if (motor >= MOTOR_COUNT)
        return 0;
    return s_channels[motor].maxDepth;
}

/* ── HAL UART TX complete callback router ────────────────────────────────────
 * Single project-wide override of the HAL weak symbol. Routes to the TX DMA
 * state owner. Kept short: no logging, no blocking, no HAL_Delay().
 * ─────────────────────────────────────────────────────────────────────────── */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    MotorTxDma_OnTxComplete(huart);
    ManipulationUartDma_OnTxComplete(huart);
}