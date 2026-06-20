#include "logger.h"
#include "app_config.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ── Private variables ──────────────────────────────────────────────────── */
static char txBuf[TERMINAL_TX_BUF_SIZE];

static const char *levelStr[] = { "INFO", "WARN", "ERROR", "DEBUG" };

/* ── Public functions ───────────────────────────────────────────────────── */

void Logger_Init(void)
{
    memset(txBuf, 0, sizeof(txBuf));
}

void Logger_Log(LogLevel_t level, const char *fmt, ...)
{
    int len = snprintf(txBuf, sizeof(txBuf), "[%s] ", levelStr[level]);

    va_list args;
    va_start(args, fmt);
    len += vsnprintf(txBuf + len, sizeof(txBuf) - len, fmt, args);
    va_end(args);

    /* Append \r\n */
    if (len < (int)sizeof(txBuf) - 2)
    {
        txBuf[len++] = '\r';
        txBuf[len++] = '\n';
    }

    HAL_UART_Transmit(&huart3, (uint8_t *)txBuf, (uint16_t)len, HAL_MAX_DELAY);
}

void Logger_LogMotorCmd(MotorId_t id, const MotorCmd_t *cmd)
{
    const char *names[] = { "FL", "FR", "RL", "RR" };
    const char *dirStr[] = { "stop", "f", "b" };
    Logger_Log(LOG_INFO, "MOTOR %s -> %s%u", names[id], dirStr[cmd->dir], cmd->pwm);
}

void Logger_LogAck(MotorId_t id, AckStatus_t status)
{
    const char *names[] = { "FL", "FR", "RL", "RR" };
    const char *statusStr[] = { "NONE", "OK", "TIMEOUT", "ERROR" };
    Logger_Log(LOG_DEBUG, "ACK %s status=%s", names[id], statusStr[status]);
}
