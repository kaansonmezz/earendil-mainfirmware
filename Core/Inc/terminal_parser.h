#ifndef TERMINAL_PARSER_H
#define TERMINAL_PARSER_H

#include "rover_types.h"

/* ── Parsed command type ─────────────────────────────────────────────────── */
typedef enum
{
    TCMD_NONE = 0,
    TCMD_HELP,          /* help */
    TCMD_STOP,          /* stop */
    TCMD_BRAKE,         /* brake → send x to all motors */
    TCMD_IDENTIFY,      /* identify */
    TCMD_STATUS,        /* status */
    TCMD_MODE_RPM,      /* mode rpm */
    TCMD_MODE_PWM,      /* mode pwm */
    TCMD_MODE_QUERY,    /* mode (print current) */
    TCMD_MOTION         /* f/b/r/l/fd/bd/rd/ld + value */
} TerminalCmdType_t;

/* ── Parse result ────────────────────────────────────────────────────────── */
typedef struct
{
    TerminalCmdType_t type;
    MotionCmd_t       motion;   /* valid when type == TCMD_MOTION */
    bool              isDuty;   /* true for fd/bd/rd/ld, false for f/b/r/l */
} TerminalResult_t;

/* ── Public API ─────────────────────────────────────────────────────────── */
bool TerminalParser_Parse(const char *line, TerminalResult_t *outResult);

#endif /* TERMINAL_PARSER_H */
