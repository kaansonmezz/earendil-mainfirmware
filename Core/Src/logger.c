#include "logger.h"
#include "app_config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ── Private variables ──────────────────────────────────────────────────── */
static char txBuf[TERMINAL_TX_BUF_SIZE];

static const char *levelStr[] = { "INFO", "WARN", "ERROR", "DEBUG", "BOOT" };
#define LOGGER_LEVEL_COUNT (sizeof(levelStr) / sizeof(levelStr[0]))

/* ── Public functions ───────────────────────────────────────────────────── */

void Logger_Init(void)
{
    memset(txBuf, 0, sizeof(txBuf));
}

void Logger_Log(LogLevel_t level, const char *fmt, ...)
{
    const size_t capacity = sizeof(txBuf);
    if (capacity < 3U)
        return;

    /* Reserve two bytes for CRLF and one for the terminating NUL.  snprintf
     * and vsnprintf return the length that *would* have been written, so
     * every return value is converted to the actual bounded length. */
    const size_t formatCapacity = capacity - 2U;
    const char *levelName = "UNKNOWN";
    if ((unsigned int)level < (unsigned int)LOGGER_LEVEL_COUNT)
        levelName = levelStr[(unsigned int)level];

    int result = snprintf(txBuf, formatCapacity, "[%s] ", levelName);
    size_t len = 0U;
    if (result < 0)
    {
        txBuf[0] = '\0';
    }
    else if ((size_t)result >= formatCapacity)
    {
        len = formatCapacity - 1U;
    }
    else
    {
        len = (size_t)result;
    }

    if (fmt != NULL && len < (formatCapacity - 1U))
    {
        size_t remaining = formatCapacity - len;
        va_list args;
        va_start(args, fmt);
        result = vsnprintf(txBuf + len, remaining, fmt, args);
        va_end(args);

        if (result >= 0)
        {
            size_t actual = ((size_t)result >= remaining)
                          ? (remaining - 1U)
                          : (size_t)result;
            len += actual;
        }
        else
        {
            txBuf[len] = '\0';
        }
    }

    /* CRLF always fits because it was reserved before formatting. */
    txBuf[len++] = '\r';
    txBuf[len++] = '\n';
    txBuf[len] = '\0';

    HAL_UART_Transmit(&huart3, (uint8_t *)txBuf, (uint16_t)len, HAL_MAX_DELAY);
}

void Logger_LogMotorCmd(MotorId_t id, const MotorCmd_t *cmd)
{
    const char *names[] = { "FL", "FR", "RL", "RR" };
    const char *dirStr[] = { "stop", "f", "b", "brk" };
    Logger_Log(LOG_INFO, "MOTOR %s -> %s%u", names[id], dirStr[cmd->dir], cmd->pwm);
}

void Logger_LogAck(MotorId_t id, AckStatus_t status)
{
    const char *names[] = { "FL", "FR", "RL", "RR" };
    const char *statusStr[] = { "NONE", "OK", "TIMEOUT", "ERROR" };
    Logger_Log(LOG_DEBUG, "ACK %s status=%s", names[id], statusStr[status]);
}
