#include "motor_tx_dma.h"
#include "app_config.h"
#include <string.h>

/* ── Configuration ────────────────────────────────────────────────────────── */
#define MOTOR_TX_DMA_BUFFER_SIZE 64U
#define MOTOR_TX_DMA_COUNT       MOTOR_COUNT

/* DMA-safe section attribute – matches the style used for RX DMA buffers. */
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

/* ── Per-motor channel state ──────────────────────────────────────────────── */
typedef struct
{
    MotorId_t           motor;
    UART_HandleTypeDef *huart;

    uint8_t            *txBuffer;
    uint16_t            txLen;
    bool                busy;

    uint8_t            *pendingBuffer;
    uint16_t            pendingLen;
    bool                pending;
    bool                pendingSafety;  /* true if pending cmd is stop/brake */
} MotorTxDmaChannel_t;

/* ── DMA-safe TX buffers (one active + one pending per motor) ─────────────── */
static uint8_t fl_tx_buffer   [MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t fr_tx_buffer   [MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t rl_tx_buffer   [MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t rr_tx_buffer   [MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;

static uint8_t fl_pending_buf [MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t fr_pending_buf [MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t rl_pending_buf [MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;
static uint8_t rr_pending_buf [MOTOR_TX_DMA_BUFFER_SIZE] DMA_BUFFER;

/* ── Channel table ──────────────────────────────────────────────────────────
 * Uses the same motor-to-UART mapping as motor_dispatcher.c / app_config.h
 *   FL -> huart2 (USART2)
 *   FR -> huart4 (UART4)
 *   RL -> huart7 (UART7)
 *   RR -> huart5 (UART5)
 * ─────────────────────────────────────────────────────────────────────────── */
static MotorTxDmaChannel_t s_channels[MOTOR_TX_DMA_COUNT] =
{
    { MOTOR_FL, &huart2, fl_tx_buffer,   0, false, fl_pending_buf, 0, false, false },
    { MOTOR_FR, &huart4, fr_tx_buffer,   0, false, fr_pending_buf, 0, false, false },
    { MOTOR_RL, &huart7, rl_tx_buffer,   0, false, rl_pending_buf, 0, false, false },
    { MOTOR_RR, &huart5, rr_tx_buffer,   0, false, rr_pending_buf, 0, false, false },
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

static void CopyCommand(uint8_t *dst, const char *cmd, uint16_t *outLen)
{
    uint16_t len = 0;
    while (cmd[len] != '\0' && len < MOTOR_TX_DMA_BUFFER_SIZE - 1U)
    {
        dst[len] = (uint8_t)cmd[len];
        len++;
    }
    dst[len] = 0U;
    *outLen  = len;
}

/* Returns true if cmd is a safety-critical payload ("stop" or "x"),
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

    if (end == 1U && cmd[0] == 'x')
        return true;

    return false;
}

static bool StartDmaTx(MotorTxDmaChannel_t *ch)
{
    if (HAL_UART_Transmit_DMA(ch->huart, ch->txBuffer, ch->txLen) == HAL_OK)
    {
        ch->busy = true;
        return true;
    }
    return false;
}

/* ── Public functions ─────────────────────────────────────────────────────── */

void MotorTxDma_Init(void)
{
    for (int i = 0; i < MOTOR_TX_DMA_COUNT; i++)
    {
        MotorTxDmaChannel_t *ch = &s_channels[i];

        ch->motor           = (MotorId_t)i;
        ch->huart           = MOTOR_UART_HANDLE((MotorId_t)i);
        ch->txLen           = 0;
        ch->pendingLen      = 0;
        ch->busy            = false;
        ch->pending         = false;
        ch->pendingSafety   = false;

        memset(ch->txBuffer,      0, MOTOR_TX_DMA_BUFFER_SIZE);
        memset(ch->pendingBuffer, 0, MOTOR_TX_DMA_BUFFER_SIZE);
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

    if (!ch->busy)
    {
        CopyCommand(ch->txBuffer, cmd, &ch->txLen);
        if (StartDmaTx(ch))
            return true;

        /* Start failed – keep state safe, do not block. */
        ch->busy = false;
        return false;
    }

    /* Channel busy – stage the command as pending with stop/brake priority.
     *
     * Safety commands (stop / x) always overwrite the pending slot.
     * Normal commands overwrite only when the current pending is NOT a
     * safety command, so a queued stop/brake is never displaced by a
     * lower-priority motion command. */
    bool newIsSafety = IsSafetyCommand(cmd, len);

    if (newIsSafety || !ch->pendingSafety)
    {
        CopyCommand(ch->pendingBuffer, cmd, &ch->pendingLen);
        ch->pending        = true;
        ch->pendingSafety  = newIsSafety;
    }

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

    ch->busy = false;

    if (ch->pending)
    {
        CopyCommand(ch->txBuffer, (const char *)ch->pendingBuffer, &ch->txLen);
        ch->pending       = false;
        ch->pendingLen    = 0;
        ch->pendingSafety = false;

        if (!StartDmaTx(ch))
            ch->busy = false;
    }
}

void MotorTxDma_OnTxError(UART_HandleTypeDef *huart)
{
    MotorTxDmaChannel_t *ch = FindChannelByUart(huart);
    if (ch == NULL)
        return;

    /* Clear busy on TX error. Pending (if any) is PRESERVED so that a
     * safety-critical stop/brake queued before the error is not lost.
     * The next successful MotorTxDma_Send() on an idle channel or a
     * future retry mechanism can flush it. Does not reset RX DMA and
     * does not interfere with the existing UART error callback policy
     * in motor_uart_dma.c. */
    ch->busy = false;
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
    return s_channels[motor].pending;
}

bool MotorTxDma_AllIdle(void)
{
    for (int i = 0; i < MOTOR_TX_DMA_COUNT; i++)
    {
        if (s_channels[i].busy || s_channels[i].pending)
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
        MotorTxDmaChannel_t *ch = &s_channels[i];
        ch->pending       = false;
        ch->pendingLen    = 0;
        ch->pendingSafety = false;
    }
}

/* ── HAL UART TX complete callback router ────────────────────────────────────
 * Single project-wide override of the HAL weak symbol. Routes to the TX DMA
 * state owner. Kept short: no logging, no blocking, no HAL_Delay().
 * ─────────────────────────────────────────────────────────────────────────── */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    MotorTxDma_OnTxComplete(huart);
}
