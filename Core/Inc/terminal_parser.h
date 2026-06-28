#ifndef TERMINAL_PARSER_H
#define TERMINAL_PARSER_H

#include "rover_types.h"

/* Max length of a raw motor-direct payload (excludes the "XX " prefix).
 * Matches MAX_LINE_LEN so any trimmed terminal line fits. */
#define RAW_PAYLOAD_MAX 61

/* ── Parsed command type ─────────────────────────────────────────────────── */
typedef enum
{
    TCMD_NONE = 0,
    TCMD_HELP,          /* help */
    TCMD_STOP,          /* stop */
    TCMD_BRAKE,         /* brake -> send x to all motors */
    TCMD_IDENTIFY,      /* identify */
    TCMD_STATUS,        /* status */
    TCMD_MODE_RPM,      /* m speed */
    TCMD_MODE_PWM,      /* m duty */
    TCMD_MODE_QUERY,    /* mode (print current rover mode) */
    TCMD_OP_MODE,       /* mode disarm / mode manual / mode auto / mode autonomous */
    TCMD_MOTION,        /* f/b/r/l/fd/bd/rd/ld + value */
    TCMD_MOTOR_RAW      /* FL/FR/RL/RR <text> : raw text to one motor only */
} TerminalCommandType_t;

/* ── Parse result ────────────────────────────────────────────────────────── */
typedef struct
{
    TerminalCommandType_t type;
    MotionCmd_t   motion;        /* direction + clamped speed (TCMD_MOTION / TCMD_STOP) */
    RoverMode_t   opMode;        /* target operating mode (TCMD_OP_MODE) */
    bool          isDuty;        /* true for fd/bd/rd/ld, false for f/b/r/l */
    uint16_t      value;         /* clamped numeric value */
    uint16_t      originalValue; /* raw numeric value before clamping */
    bool          hasValue;      /* true if a numeric value was parsed */
    bool          wasClamped;    /* true if value was clamped to its allowed range */

    /* TCMD_MOTOR_RAW: target motor and raw text payload.  An empty payload
     * (rawPayload[0] == '\0') means the user typed only the bare motor tag
     * (e.g. "FL"); the handler must emit a usage error.  The payload never
     * includes the "XX " prefix nor any trailing CR/LF. */
    MotorId_t     rawMotor;
    char          rawPayload[RAW_PAYLOAD_MAX];
} TerminalCommand_t;

/* ── Public API ─────────────────────────────────────────────────────────── */
bool TerminalParser_Parse(const char *line, TerminalCommand_t *outResult);

#endif /* TERMINAL_PARSER_H */
