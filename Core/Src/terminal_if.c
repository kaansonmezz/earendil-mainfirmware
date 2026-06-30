#include "terminal_if.h"
#include "app_config.h"
#include <string.h>

/* ── Configuration ────────────────────────────────────────────────────────── */
#define TERMINAL_LINE_QUEUE_DEPTH 16U

/* ── Private variables ───────────────────────────────────────────────────────
 * Byte-by-byte accumulation happens in s_rxBuildBuf.  When a newline arrives
 * the completed line is copied into a FIFO of completed lines (s_lineQueue).
 * The main loop pops lines from the FIFO via TerminalIf_GetLine(), so a burst
 * of commands from the GUI can never overwrite an unprocessed earlier line. */

static char     s_rxBuildBuf[TERMINAL_RX_BUF_SIZE];
static uint16_t s_rxBuildPos;

static char     s_lineQueue[TERMINAL_LINE_QUEUE_DEPTH][TERMINAL_RX_BUF_SIZE];

static volatile uint8_t  s_qHead;
static volatile uint8_t  s_qTail;
static volatile uint8_t  s_qCount;

static volatile uint32_t s_qDropped;
static volatile uint32_t s_qReceived;
static volatile uint8_t  s_qMaxDepth;

static uint8_t rxByte;

/* ── Queue helpers ──────────────────────────────────────────────────────────
 * The queue is written from RX callback (ISR) context and read from the main
 * loop.  Brief IRQ disable sections keep the head/tail/count/counters
 * consistent across the two contexts. */

static bool QueuePushFromIsr(const char *line)
{
    if (line == NULL || line[0] == '\0')
        return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_qCount >= TERMINAL_LINE_QUEUE_DEPTH)
    {
        s_qDropped++;
        if (!primask) __enable_irq();
        return false;
    }

    char *slot = s_lineQueue[s_qTail];
    /* Bounded copy — line is already null-terminated and <= buf-1. */
    uint16_t i = 0;
    while (line[i] != '\0' && i < (TERMINAL_RX_BUF_SIZE - 1U))
    {
        slot[i] = line[i];
        i++;
    }
    slot[i] = '\0';

    s_qTail = (uint8_t)((s_qTail + 1U) % TERMINAL_LINE_QUEUE_DEPTH);
    s_qCount++;
    s_qReceived++;

    if (s_qCount > s_qMaxDepth)
        s_qMaxDepth = s_qCount;

    if (!primask) __enable_irq();
    return true;
}

static bool QueuePop(char *out, size_t outSize)
{
    if (out == NULL || outSize == 0U)
        return false;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_qCount == 0U)
    {
        if (!primask) __enable_irq();
        return false;
    }

    const char *src = s_lineQueue[s_qHead];
    size_t i = 0;
    while (src[i] != '\0' && i < (outSize - 1U))
    {
        out[i] = src[i];
        i++;
    }
    out[i] = '\0';

    s_qHead = (uint8_t)((s_qHead + 1U) % TERMINAL_LINE_QUEUE_DEPTH);
    s_qCount--;

    if (!primask) __enable_irq();
    return true;
}

static void QueueClear(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_qHead  = 0U;
    s_qTail  = 0U;
    s_qCount = 0U;
    if (!primask) __enable_irq();
}

/* ── Public functions ─────────────────────────────────────────────────────── */

void TerminalIf_Init(void)
{
    s_rxBuildPos = 0;
    memset(s_rxBuildBuf, 0, sizeof(s_rxBuildBuf));
    memset(s_lineQueue,  0, sizeof(s_lineQueue));

    s_qHead     = 0U;
    s_qTail     = 0U;
    s_qCount    = 0U;
    s_qDropped  = 0U;
    s_qReceived = 0U;
    s_qMaxDepth = 0U;

    /* Start interrupt-based single-byte reception on USART3 */
    HAL_UART_Receive_IT(&huart3, &rxByte, 1);
}

void TerminalIf_Process(void)
{
    /* Called from main loop – nothing heavy here.
       Lines are popped by the caller via TerminalIf_GetLine(). */
}

uint8_t TerminalIf_RxCallback(uint8_t byte)
{
    if (byte == '\r')
    {
        return 0; /* ignore CR */
    }

    if (byte == '\n')
    {
        if (s_rxBuildPos > 0U)
        {
            s_rxBuildBuf[s_rxBuildPos] = '\0';
            QueuePushFromIsr(s_rxBuildBuf);
        }
        s_rxBuildPos = 0U;
        return 1; /* line complete */
    }

    if (s_rxBuildPos < (TERMINAL_RX_BUF_SIZE - 1U))
    {
        s_rxBuildBuf[s_rxBuildPos++] = (char)byte;
    }
    else
    {
        /* Line too long — drop/reset safely without overflowing. */
        s_rxBuildPos = 0U;
        s_qDropped++;
    }

    return 0;
}

bool TerminalIf_GetLine(char *outLine, size_t outSize)
{
    return QueuePop(outLine, outSize);
}

uint8_t TerminalIf_GetPendingLineCount(void)
{
    return s_qCount;
}

uint32_t TerminalIf_GetDroppedLineCount(void)
{
    return s_qDropped;
}

uint32_t TerminalIf_GetReceivedLineCount(void)
{
    return s_qReceived;
}

uint8_t TerminalIf_GetMaxLineQueueDepth(void)
{
    return s_qMaxDepth;
}

/* ── Legacy single-line API (deprecated but kept for compatibility) ────────
 * These are no longer used by app_main.c but are kept so external code that
 * still references them compiles.  They operate on the new FIFO internally. */

bool TerminalIf_LineReady(void)
{
    return s_qCount > 0U;
}

const char *TerminalIf_GetLinePtr(void)
{
    /* Not safe for multi-consumer use; callers should use TerminalIf_GetLine. */
    static char legacyBuf[TERMINAL_RX_BUF_SIZE];
    if (QueuePop(legacyBuf, sizeof(legacyBuf)))
        return legacyBuf;
    return NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        TerminalIf_RxCallback(rxByte);
        HAL_UART_Receive_IT(&huart3, &rxByte, 1);
    }
}