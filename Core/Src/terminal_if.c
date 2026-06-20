#include "terminal_if.h"
#include "app_config.h"
#include <string.h>

/* ── Private variables ──────────────────────────────────────────────────── */
static char     lineBuf[TERMINAL_RX_BUF_SIZE];
static uint16_t linePos = 0;
static volatile bool lineReady = false;
static uint8_t  rxByte;

/* ── Public functions ───────────────────────────────────────────────────── */

void TerminalIf_Init(void)
{
    linePos = 0;
    lineReady = false;
    memset(lineBuf, 0, sizeof(lineBuf));

    /* Start interrupt-based single-byte reception on USART3 */
    HAL_UART_Receive_IT(&huart3, &rxByte, 1);
}

void TerminalIf_Process(void)
{
    /* Called from main loop – nothing heavy here yet.
       Line-ready flag is checked by caller via TerminalIf_LineReady(). */
}

uint8_t TerminalIf_RxCallback(uint8_t byte)
{
    if (byte == '\r' || byte == '\n')
    {
        if (linePos > 0)
        {
            lineBuf[linePos] = '\0';
            lineReady = true;
            linePos = 0;
            return 1; /* line complete */
        }
        return 0; /* empty line, ignore */
    }

    if (linePos < TERMINAL_RX_BUF_SIZE - 1)
    {
        lineBuf[linePos++] = (char)byte;
    }

    return 0;
}

bool TerminalIf_LineReady(void)
{
    return lineReady;
}

const char *TerminalIf_GetLine(void)
{
    lineReady = false;
    return lineBuf;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        TerminalIf_RxCallback(rxByte);
        HAL_UART_Receive_IT(&huart3, &rxByte, 1);
    }
}
