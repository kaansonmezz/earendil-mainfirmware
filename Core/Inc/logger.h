#ifndef LOGGER_H
#define LOGGER_H

#include "rover_types.h"

/* ── Log levels ─────────────────────────────────────────────────────────── */
typedef enum
{
    LOG_INFO = 0,
    LOG_WARN,
    LOG_ERROR,
    LOG_DEBUG,
    LOG_BOOT
} LogLevel_t;

/* ── Public API ─────────────────────────────────────────────────────────── */
void     Logger_Init(void);
void     Logger_Log(LogLevel_t level, const char *fmt, ...);
void     Logger_LogMotorCmd(MotorId_t id, const MotorCmd_t *cmd);
void     Logger_LogAck(MotorId_t id, AckStatus_t status);

#endif /* LOGGER_H */
