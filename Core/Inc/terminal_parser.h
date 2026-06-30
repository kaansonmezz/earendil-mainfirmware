#ifndef TERMINAL_PARSER_H
#define TERMINAL_PARSER_H

#include "rover_types.h"

/* Max length of a raw motor-direct payload (excludes the "XX " prefix).
 * Matches MAX_LINE_LEN so any trimmed terminal line fits. */
#define RAW_PAYLOAD_MAX 61

/* Max length of a normalised tune payload forwarded to F411.
 * e.g. "base 40 40 45 45 50 50 55 55" = 35 chars + NUL. */
#define TUNE_PAYLOAD_MAX 56

/* ── Motor target for tune commands ──────────────────────────────────────── */
typedef enum
{
    TUNE_MOTOR_NONE = 0,
    TUNE_MOTOR_FL,
    TUNE_MOTOR_FR,
    TUNE_MOTOR_RL,
    TUNE_MOTOR_RR,
    TUNE_MOTOR_ALL
} TuneMotorTarget_t;

/* ── Tune command kind ───────────────────────────────────────────────────── */
typedef enum
{
    TUNE_KIND_NONE = 0,
    TUNE_KIND_BASE,       /* base P1..P8                    -> "base P1..P8"           */
    TUNE_KIND_BOOST,      /* boost P1..P8 MS                -> "boost P1..P8 MS"       */
    TUNE_KIND_KICKDUTY,   /* kickduty / kick duty VALUE     -> "kickduty VALUE"         */
    TUNE_KIND_KICKMS,     /* kickms   / kick ms VALUE       -> "kickms VALUE"           */
    TUNE_KIND_RAMP,       /* ramp UP DOWN                   -> "ramp UP DOWN"           */
    TUNE_KIND_PI,         /* pi KP KI                       -> "pi KP KI"              */
    TUNE_KIND_TELPER      /* telper MS                      -> "telper MS"             */
} TuneCmdKind_t;

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
    TCMD_MOTOR_RAW,     /* FL/FR/RL/RR <text> : raw text to one motor only */
    TCMD_MOTOR_TUNE,    /* FL/FR/RL/RR/ALL <tuning command> : validated tuning */
    TCMD_TERMSTAT       /* termstat : terminal RX queue diagnostics */
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

    /* TCMD_MOTOR_TUNE: validated tuning command fields.
     * tuneTarget — which motor(s) to target (FL/FR/RL/RR/ALL).
     * tuneKind   — which tuning command (base/boost/kickduty/…).
     * tunePayload — normalised string to forward to F411 UART,
     *               e.g. "base 40 40 45 45 50 50 55 55".
     *               Does NOT include "\r\n" — the dispatcher adds it. */
    TuneMotorTarget_t tuneTarget;
    TuneCmdKind_t     tuneKind;
    char              tunePayload[TUNE_PAYLOAD_MAX];
} TerminalCommand_t;

/* ── Public API ─────────────────────────────────────────────────────────── */
bool TerminalParser_Parse(const char *line, TerminalCommand_t *outResult);

#endif /* TERMINAL_PARSER_H */
