#ifndef TERMINAL_PARSER_H
#define TERMINAL_PARSER_H

#include "rover_types.h"

/* ── Parsed command type ─────────────────────────────────────────────────── */
typedef enum
{
    TCMD_NONE = 0,
    TCMD_HELP,          /* help */
    TCMD_STOP,          /* stop */
    TCMD_BRAKE,         /* brake -> send x to all motors */
    TCMD_IDENTIFY,      /* identify */
    TCMD_STATUS,        /* status */
    TCMD_MODE_RPM,      /* m rpm */
    TCMD_MODE_PWM,      /* m pwm */
    TCMD_MODE_QUERY,    /* mode (print current rover mode) */
    TCMD_MOTION         /* f/b/r/l/fd/bd/rd/ld + value */
} TerminalCommandType_t;

/* ── Parse result ────────────────────────────────────────────────────────── */
typedef struct
{
    TerminalCommandType_t type;
    MotionCmd_t   motion;        /* direction + clamped speed (TCMD_MOTION / TCMD_STOP) */
    bool          isDuty;        /* true for fd/bd/rd/ld, false for f/b/r/l */
    uint16_t      value;         /* clamped numeric value */
    uint16_t      originalValue; /* raw numeric value before clamping */
    bool          hasValue;      /* true if a numeric value was parsed */
    bool          wasClamped;    /* true if value was clamped to its allowed range */
} TerminalCommand_t;

/* ── Public API ─────────────────────────────────────────────────────────── */
bool TerminalParser_Parse(const char *line, TerminalCommand_t *outResult);

#endif /* TERMINAL_PARSER_H */
