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

/* ── Drive arc-turn motion direction ──────────────────────────────────────── */
typedef enum
{
    DRIVE_ARC_NONE = 0,
    DRIVE_ARC_FL,   /* forward-left  arc turn */
    DRIVE_ARC_FR,   /* forward-right arc turn */
    DRIVE_ARC_BL,   /* backward-left  arc turn */
    DRIVE_ARC_BR    /* backward-right arc turn */
} DriveArcMotion_t;

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
    TCMD_TERMSTAT,      /* termstat : terminal RX queue diagnostics */
    TCMD_I2CSCAN,       /* i2cscan : scan I2C1 bus for devices */
    TCMD_MPUWHO,        /* mpuwho : read MPU9250 WHO_AM_I register */
    TCMD_MPUREGS,       /* mpuregs : read MPU9250 diagnostic registers */
    TCMD_MPUWARM,       /* mpuwarm : probe before/after warm-up only */
    TCMD_MPUINIT,       /* mpuinit : basic MPU6500/9250 init */
    TCMD_MPUCFGTEST,    /* mpucfgtest : CONFIG register write/readback diagnostic */
    TCMD_MPURAW,        /* mpuraw : one-shot raw accel/gyro/temperature read */
    TCMD_MPUDDBGRAW,    /* mpudbgraw : update IMU debug variables for CubeIDE */
    TCMD_MPUGYROTEST,   /* mpugyrotest : gyro-specific diagnostic */
    TCMD_MPUCONV,       /* mpuconv : converted accel/gyro/temp in physical units */
    TCMD_MPUBIAS,       /* mpubias : query gyro bias state */
    TCMD_MPUBIASON,     /* mpubiason : enable gyro bias correction */
    TCMD_MPUBIASOFF,    /* mpubiasoff : disable gyro bias correction */
    TCMD_MPUBIASCLEAR,  /* mpubiasclear : clear gyro bias to zero */
    TCMD_IMU_HELP,      /* imu help : show IMU command list */
    TCMD_IMU_STREAM_ON, /* imu stream on : enable periodic IMU telemetry */
    TCMD_IMU_STREAM_OFF,/* imu stream off : disable periodic IMU telemetry */
    TCMD_IMU_TELPER,    /* imu telper <ms> : set IMU telemetry period */
    TCMD_IMU_GYROFILTER_STATUS, /* imu gyrofilter status */
    TCMD_IMU_GYROFILTER_ON,     /* imu gyrofilter on */
    TCMD_IMU_GYROFILTER_OFF,    /* imu gyrofilter off */
    TCMD_IMU_DEADBAND,          /* imu deadband <mdps> */
    TCMD_IMU_LPF,               /* imu lpf <alpha_permille> */
    TCMD_MAGWHO,                /* magwho : detect QMC5883L magnetometer */
    TCMD_MAGINIT,               /* maginit : init QMC5883L magnetometer */
    TCMD_MAGRAW,                /* magraw : read raw magnetometer X/Y/Z */
    TCMD_MAGIMU,                /* magimu : read compact GUI-friendly magnetometer X/Y/Z */
    TCMD_MAGHELP,               /* maghelp : show magnetometer commands */
    TCMD_MAGSTATUS,             /* magstatus : full magnetometer diagnostic status */
    TCMD_MAG_TELPER,            /* mag telper <ms> : set magnetometer telemetry period */
    TCMD_GYRO_TELPER,           /* gyro telper <ms> : set gyroscope telemetry period */
    TCMD_ACCEL_TELPER,          /* accel telper <ms> : set accelerometer telemetry period */
    TCMD_DRIVE_ARC,             /* drive <rpm|duty> <target> <fl|fr|bl|br> tr <decimal> */
    TCMD_CFGCACHE,              /* cfgcache [FL|FR|RL|RR] : print cached tuning config */
    TCMD_CFGREAD,               /* cfgread FL|FR|RL|RR|all : request cfg from motor(s) */
    TCMD_ARM_RAW,               /* arm <payload> : raw text to manipulation F411 */
    TCMD_HB,                    /* hb / heartbeat : PC control-link keepalive */
    TCMD_LINKSTAT               /* linkstat : diagnostic control-link status */
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

    /* TCMD_DRIVE_ARC: arc-turn drive command fields.
     * driveIsDuty          — true for "drive duty", false for "drive rpm".
     * driveTarget          — target RPM (0..200) or duty (0..4000).
     * driveTurnRatioPermille — turn ratio as fixed-point 0..1000.
     * driveMotion          — which arc direction (FL/FR/BL/BR). */
    bool              driveIsDuty;
    uint16_t          driveTarget;
    uint16_t          driveTurnRatioPermille;
    DriveArcMotion_t  driveMotion;

    /* TCMD_CFGCACHE / TCMD_CFGREAD: target motor.
     * MOTOR_COUNT means "all motors" (bare cfgcache / cfgread all). */
    MotorId_t         cfgMotor;

    /* TCMD_ARM_RAW: raw payload to forward to manipulation F411 over UART8.
     * armPayload[0] == '\0' means bare "arm" with no payload — handler must
     * emit a usage error.  The payload never includes the "arm " prefix nor
     * any trailing CR/LF.  Case is preserved exactly as typed. */
    char              armPayload[RAW_PAYLOAD_MAX];
} TerminalCommand_t;

/* ── Public API ─────────────────────────────────────────────────────────── */
bool TerminalParser_Parse(const char *line, TerminalCommand_t *outResult);

#endif /* TERMINAL_PARSER_H */
