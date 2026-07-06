#include "motor_tuning_config.h"
#include "app_config.h"
#include "logger.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* ── Per-motor config cache ─────────────────────────────────────────────── */
static MotorTuningConfig_t s_cfg[MOTOR_COUNT];

/* ── Pending capture buffer (written during a cfg read, committed on
 *    completion).  Separate from s_cfg so an incomplete read never
 *    corrupts the last valid cache. ──────────────────────────────────── */
static MotorTuningConfig_t s_pending[MOTOR_COUNT];

/* ── Slot-to-MotorId mapping ──────────────────────────────────────────────
 *  Slot ordering in motor_uart_dma.c:
 *    0 = USART2 -> FL
 *    1 = UART4  -> FR
 *    2 = UART5  -> RR
 *    3 = UART7  -> RL                                             */
static const MotorId_t s_slotToMotor[4] = {
    MOTOR_FL, MOTOR_FR, MOTOR_RR, MOTOR_RL
};

MotorId_t MotorTuningConfig_SlotToMotorId(int slot)
{
    if (slot >= 0 && slot < 4)
        return s_slotToMotor[slot];
    return MOTOR_FL; /* fallback, should not happen */
}

/* ── Helpers ─────────────────────────────────────────────────────────────── */

static const char *MotorTag(MotorId_t id)
{
    switch (id)
    {
        case MOTOR_FL: return "FL";
        case MOTOR_FR: return "FR";
        case MOTOR_RL: return "RL";
        case MOTOR_RR: return "RR";
        default:       return "??";
    }
}

/* Skip leading whitespace, return pointer to first non-space. */
static const char *skip_ws(const char *s)
{
    while (*s == ' ' || *s == '\t')
        s++;
    return s;
}

/* Parse an unsigned integer from the string, advance *pp past it.
 * Returns true on success, false if no digits found or value out of range. */
static bool parse_u16(const char **pp, uint16_t *out)
{
    const char *p = *pp;
    p = skip_ws(p);
    if (!isdigit((unsigned char)*p))
        return false;

    unsigned long v = strtoul(p, (char **)&p, 10);
    if (v > 65535UL)
        return false;

    *out = (uint16_t)v;
    *pp = p;
    return true;
}

static bool parse_i32(const char **pp, int32_t *out)
{
    const char *p = *pp;
    p = skip_ws(p);

    char *end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p)
        return false;

    *out = (int32_t)v;
    *pp = end;
    return true;
}

/* ── Line classifiers ───────────────────────────────────────────────────── */

/* Returns true if the line is telemetry and must NOT update config cache.
 * Telemetry is identified by the payload starting with "RPM:" — a loose
 * strstr("RPM:") would incorrectly classify mixed error+telemetry lines
 * like "[ERR] Unknown commandRPM:0,..." as telemetry. */
static bool is_telemetry(const char *line)
{
    /* F411 compact telemetry: payload starts with RPM: */
    if (strncmp(line, "RPM:", 4) == 0 &&
        strstr(line, "PWM_ACT:") != NULL &&
        strstr(line, "RXB:") != NULL)
        return true;

    /* [TEL][MOTOR] prefixed telemetry (already tagged by H7) */
    if (strncmp(line, "[TEL][", 6) == 0)
        return true;

    /* [OK] RPM=... or [OK] Stop */
    if (strncmp(line, "[OK]", 4) == 0)
        return true;

    /* Link status */
    if (strstr(line, "LINK_LOST") != NULL || strstr(line, "LINK_RECOVERED") != NULL)
        return true;

    return false;
}

/* Returns true if the line should be silently ignored (not cfg, not telemetry).
 * [ERR] lines are NOT ignored here — they are checked for unsupported cfg
 * in the main parser. */
static bool should_ignore(const char *line)
{
    /* Empty */
    if (line[0] == '\0')
        return true;

    return false;
}

/* ── Detect [ERR] Unknown command from F411 ──────────────────────────────── */
static bool try_detect_cfg_error(const char *line, MotorId_t motor,
                                  MotorTuningConfig_t *cfg)
{
    /* Pattern: [ERR] Unknown command */
    if (strncmp(line, "[ERR]", 5) != 0)
        return false;

    const char *p = skip_ws(line + 5);

    if (strncmp(p, "Unknown command", 15) == 0)
    {
        strncpy(cfg->last_error, "unsupported cfg", CFG_ERROR_MAX - 1);
        cfg->last_error[CFG_ERROR_MAX - 1] = '\0';
        cfg->valid = 0;
        Logger_Log(LOG_WARN, "[CFG][%s] cfg rejected / unsupported firmware",
                   MotorTag(motor));
        return true;
    }

    /* Other [ERR] lines — store generic error, keep valid unchanged */
    strncpy(cfg->last_error, "f411 error", CFG_ERROR_MAX - 1);
    cfg->last_error[CFG_ERROR_MAX - 1] = '\0';
    return false;
}

/* ── Config line parsers (write to pending buffer) ──────────────────────── */

static bool try_parse_kp_ki(const char *line, MotorId_t motor)
{
    /* Pattern: Kp_m=<int> Ki_m=<int> */
    const char *p = line;

    if (strncmp(p, "Kp_m=", 5) != 0)
        return false;
    p += 5;

    int32_t kp;
    if (!parse_i32(&p, &kp))
        return false;

    p = skip_ws(p);
    if (strncmp(p, "Ki_m=", 5) != 0)
        return false;
    p += 5;

    int32_t ki;
    if (!parse_i32(&p, &ki))
        return false;

    /* Start a new pending capture — clear previous partial state */
    MotorTuningConfig_t *pend = &s_pending[motor];
    memset(pend, 0, sizeof(*pend));
    pend->kp_m = kp;
    pend->ki_m = ki;
    pend->has_pi = 1;

    Logger_Log(LOG_INFO, "[CFG][%s] PI received Kp_m=%ld Ki_m=%ld",
               MotorTag(motor), (long)kp, (long)ki);
    return true;
}

static bool try_parse_base(const char *line, MotorId_t motor)
{
    /* Pattern: Base <v1> <v2> ... <v8> */
    if (strncmp(line, "Base", 4) != 0)
        return false;
    if (line[4] != ' ' && line[4] != '\t')
        return false;

    const char *p = line + 4;
    uint16_t vals[TUNING_SLOTS];
    for (int i = 0; i < TUNING_SLOTS; i++)
    {
        if (!parse_u16(&p, &vals[i]))
        {
            Logger_Log(LOG_WARN, "[CFG][%s] invalid Base line",
                       MotorTag(motor));
            return false;
        }
    }

    /* All 8 parsed — store in pending */
    MotorTuningConfig_t *pend = &s_pending[motor];
    memcpy(pend->base_pwm, vals, sizeof(vals));
    pend->has_base = 1;

    Logger_Log(LOG_INFO, "[CFG][%s] Base received", MotorTag(motor));
    return true;
}

static bool try_parse_boost(const char *line, MotorId_t motor)
{
    /* Pattern: Boost <v1> <v2> ... <v8> ms=<ms> */
    if (strncmp(line, "Boost", 5) != 0)
        return false;
    if (line[5] != ' ' && line[5] != '\t')
        return false;

    const char *p = line + 5;
    uint16_t vals[TUNING_SLOTS];
    for (int i = 0; i < TUNING_SLOTS; i++)
    {
        if (!parse_u16(&p, &vals[i]))
        {
            Logger_Log(LOG_WARN, "[CFG][%s] invalid Boost line",
                       MotorTag(motor));
            return false;
        }
    }

    p = skip_ws(p);
    if (strncmp(p, "ms=", 3) != 0)
    {
        Logger_Log(LOG_WARN, "[CFG][%s] Boost missing ms=", MotorTag(motor));
        return false;
    }
    p += 3;

    uint16_t ms;
    if (!parse_u16(&p, &ms))
    {
        Logger_Log(LOG_WARN, "[CFG][%s] invalid Boost ms value",
                   MotorTag(motor));
        return false;
    }

    /* All valid — store in pending */
    MotorTuningConfig_t *pend = &s_pending[motor];
    memcpy(pend->boost_pwm, vals, sizeof(vals));
    pend->boost_ms = ms;
    pend->has_boost = 1;

    Logger_Log(LOG_INFO, "[CFG][%s] Boost received ms=%u",
               MotorTag(motor), (unsigned)ms);
    return true;
}

/* ── Optional parsers (Ramp / Kick / TelPer) ─────────────────────────────
 *  These write directly to the committed cache (s_cfg) because they may
 *  arrive after try_commit has already fired.  They also write to
 *  s_pending so the GUI debounce path can pick them up. */

static bool try_parse_ramp(const char *line, MotorId_t motor)
{
    /* Pattern: Ramp up=<u16> down=<u16> */
    if (strncmp(line, "Ramp", 4) != 0)
        return false;
    if (line[4] != ' ' && line[4] != '\t')
        return false;

    const char *p = line + 4;
    p = skip_ws(p);
    if (strncmp(p, "up=", 3) != 0)
        return false;
    p += 3;

    uint16_t up;
    if (!parse_u16(&p, &up))
        return false;

    p = skip_ws(p);
    if (strncmp(p, "down=", 5) != 0)
        return false;
    p += 5;

    uint16_t down;
    if (!parse_u16(&p, &down))
        return false;

    s_pending[motor].ramp_up   = up;
    s_pending[motor].ramp_down = down;
    s_cfg[motor].ramp_up       = up;
    s_cfg[motor].ramp_down     = down;

    Logger_Log(LOG_INFO, "[CFG][%s] Ramp up=%u down=%u",
               MotorTag(motor), (unsigned)up, (unsigned)down);
    return true;
}

static bool try_parse_kick(const char *line, MotorId_t motor)
{
    /* Pattern: Kick ON|OFF duty=<u16> ms=<u16> */
    if (strncmp(line, "Kick", 4) != 0)
        return false;
    if (line[4] != ' ' && line[4] != '\t')
        return false;

    const char *p = line + 4;
    p = skip_ws(p);

    uint8_t enabled = 0xFF;
    if (strncmp(p, "ON", 2) == 0 && (p[2] == ' ' || p[2] == '\t'))
    {
        enabled = 1;
        p += 2;
    }
    else if (strncmp(p, "OFF", 3) == 0 && (p[3] == ' ' || p[3] == '\t'))
    {
        enabled = 0;
        p += 3;
    }
    else
    {
        return false;
    }

    p = skip_ws(p);
    if (strncmp(p, "duty=", 5) != 0)
        return false;
    p += 5;

    uint16_t duty;
    if (!parse_u16(&p, &duty))
        return false;

    p = skip_ws(p);
    if (strncmp(p, "ms=", 3) != 0)
        return false;
    p += 3;

    uint16_t ms;
    if (!parse_u16(&p, &ms))
        return false;

    s_pending[motor].kick_enabled = enabled;
    s_pending[motor].kick_duty    = duty;
    s_pending[motor].kick_ms      = ms;
    s_cfg[motor].kick_enabled     = enabled;
    s_cfg[motor].kick_duty        = duty;
    s_cfg[motor].kick_ms          = ms;

    Logger_Log(LOG_INFO, "[CFG][%s] Kick %s duty=%u ms=%u",
               MotorTag(motor),
               enabled ? "ON" : "OFF",
               (unsigned)duty, (unsigned)ms);
    return true;
}

static bool try_parse_telper(const char *line, MotorId_t motor)
{
    /* Pattern: TelPer=<u16> */
    if (strncmp(line, "TelPer=", 7) != 0)
        return false;

    const char *p = line + 7;
    uint16_t val;
    if (!parse_u16(&p, &val))
        return false;

    s_pending[motor].telper = val;
    s_cfg[motor].telper     = val;

    Logger_Log(LOG_INFO, "[CFG][%s] TelPer=%u", MotorTag(motor), (unsigned)val);
    return true;
}

/* ── Commit pending → valid cache ────────────────────────────────────────── */

static void try_commit(MotorId_t motor)
{
    MotorTuningConfig_t *pend = &s_pending[motor];

    if (!pend->has_pi || !pend->has_base || !pend->has_boost)
        return;

    /* All three parts present — commit to the real cache */
    MotorTuningConfig_t *cfg = &s_cfg[motor];

    cfg->kp_m      = pend->kp_m;
    cfg->ki_m      = pend->ki_m;
    memcpy(cfg->base_pwm,  pend->base_pwm,  sizeof(cfg->base_pwm));
    memcpy(cfg->boost_pwm, pend->boost_pwm, sizeof(cfg->boost_pwm));
    cfg->boost_ms  = pend->boost_ms;

    /* Copy optional fields if they were parsed before commit */
    if (pend->ramp_up || pend->ramp_down)
    {
        cfg->ramp_up   = pend->ramp_up;
        cfg->ramp_down = pend->ramp_down;
    }
    if (pend->kick_duty || pend->kick_ms)
    {
        cfg->kick_duty    = pend->kick_duty;
        cfg->kick_ms      = pend->kick_ms;
        cfg->kick_enabled = pend->kick_enabled;
    }
    if (pend->telper)
    {
        cfg->telper = pend->telper;
    }
    cfg->has_pi    = 1;
    cfg->has_base  = 1;
    cfg->has_boost = 1;
    cfg->valid     = 1;
    cfg->update_count++;
    cfg->last_update_ms = HAL_GetTick();
    cfg->last_error[0] = '\0';

    /* Log with integer fixed-point (no float printf) */
    int32_t kp_int  = cfg->kp_m / 1000;
    int32_t kp_frac = cfg->kp_m % 1000; if (kp_frac < 0) kp_frac = -kp_frac;
    int32_t ki_int  = cfg->ki_m / 1000;
    int32_t ki_frac = cfg->ki_m % 1000; if (ki_frac < 0) ki_frac = -ki_frac;

    Logger_Log(LOG_INFO, "[CFG][%s] valid=1 updated Kp=%ld.%03ld Ki=%ld.%03ld boost_ms=%u",
               MotorTag(motor),
               (long)kp_int, (long)kp_frac,
               (long)ki_int, (long)ki_frac,
               (unsigned)cfg->boost_ms);

    /* Clear pending so stale data is not re-committed */
    memset(pend, 0, sizeof(*pend));
}

/* ── Public API ──────────────────────────────────────────────────────────── */

void MotorTuningConfig_Init(void)
{
    memset(s_cfg, 0, sizeof(s_cfg));
    memset(s_pending, 0, sizeof(s_pending));
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        s_cfg[i].kick_enabled = 0xFF;
        s_pending[i].kick_enabled = 0xFF;
    }
}

void MotorTuningConfig_ProcessLine(MotorId_t motor, const char *line)
{
    if (motor >= MOTOR_COUNT)
        return;

    /* Skip telemetry — must not touch config cache */
    if (is_telemetry(line))
        return;

    /* Skip empty lines */
    if (should_ignore(line))
        return;

    MotorTuningConfig_t *cfg = &s_cfg[motor];

    /* Detect [ERR] Unknown command — marks motor as unsupported */
    if (try_detect_cfg_error(line, motor, cfg))
        return;

    /* Try each cfg pattern.  Each parser writes to s_pending, not s_cfg. */
    if (try_parse_kp_ki(line, motor))
    {
        try_commit(motor);
        return;
    }

    if (try_parse_base(line, motor))
    {
        try_commit(motor);
        return;
    }

    if (try_parse_boost(line, motor))
    {
        try_commit(motor);
        return;
    }

    /* Optional parsers — write to both pending and committed cache.
     * These may arrive after try_commit has already fired. */
    if (try_parse_ramp(line, motor))
        return;

    if (try_parse_kick(line, motor))
        return;

    if (try_parse_telper(line, motor))
        return;

    /* Line did not match any cfg pattern — silently ignore.
     * This covers: status lines, unknown F411 output, etc. */
}

const MotorTuningConfig_t *MotorTuningConfig_Get(MotorId_t motor)
{
    if (motor >= MOTOR_COUNT)
        return NULL;
    return &s_cfg[motor];
}

/* ── Integer fixed-point helper ────────────────────────────────────────────
 *  Prints a fixed-point value as "N.FFF" without using float printf.
 *  Handles negative values correctly. */
static void log_fixed_point(const char *prefix, int32_t raw)
{
    int32_t int_part  = raw / 1000;
    int32_t frac_part = raw % 1000;
    if (frac_part < 0)
        frac_part = -frac_part;
    /* Build the string inline so we can use Logger_Log once */
    Logger_Log(LOG_INFO, "%s=%ld.%03ld", prefix, (long)int_part, (long)frac_part);
}

void MotorTuningConfig_Print(MotorId_t motor)
{
    if (motor >= MOTOR_COUNT)
        return;

    const MotorTuningConfig_t *cfg = &s_cfg[motor];

    if (!cfg->valid)
    {
        if (cfg->last_error[0] != '\0')
        {
            Logger_Log(LOG_INFO, "[CFG][%s] not valid / last_error=%s",
                       MotorTag(motor), cfg->last_error);
        }
        else
        {
            Logger_Log(LOG_INFO, "[CFG][%s] not valid / no cfg received yet",
                       MotorTag(motor));
        }
        return;
    }

    uint32_t age = HAL_GetTick() - cfg->last_update_ms;

    Logger_Log(LOG_INFO, "[CFG][%s] valid=%u updates=%lu age_ms=%lu",
               MotorTag(motor),
               (unsigned)cfg->valid,
               (unsigned long)cfg->update_count,
               (unsigned long)age);

    /* Kp/Ki with integer fixed-point — no float printf */
    int32_t kp_int  = cfg->kp_m / 1000;
    int32_t kp_frac = cfg->kp_m % 1000; if (kp_frac < 0) kp_frac = -kp_frac;
    int32_t ki_int  = cfg->ki_m / 1000;
    int32_t ki_frac = cfg->ki_m % 1000; if (ki_frac < 0) ki_frac = -ki_frac;

    Logger_Log(LOG_INFO, "[CFG][%s] Kp_m=%ld Ki_m=%ld Kp=%ld.%03ld Ki=%ld.%03ld",
               MotorTag(motor),
               (long)cfg->kp_m, (long)cfg->ki_m,
               (long)kp_int, (long)kp_frac,
               (long)ki_int, (long)ki_frac);

    Logger_Log(LOG_INFO, "[CFG][%s] Base %u %u %u %u %u %u %u %u",
               MotorTag(motor),
               cfg->base_pwm[0], cfg->base_pwm[1],
               cfg->base_pwm[2], cfg->base_pwm[3],
               cfg->base_pwm[4], cfg->base_pwm[5],
               cfg->base_pwm[6], cfg->base_pwm[7]);

    Logger_Log(LOG_INFO, "[CFG][%s] Boost %u %u %u %u %u %u %u %u ms=%u",
               MotorTag(motor),
               cfg->boost_pwm[0], cfg->boost_pwm[1],
               cfg->boost_pwm[2], cfg->boost_pwm[3],
               cfg->boost_pwm[4], cfg->boost_pwm[5],
               cfg->boost_pwm[6], cfg->boost_pwm[7],
               (unsigned)cfg->boost_ms);

    /* Optional fields — only print if non-zero */
    if (cfg->ramp_up || cfg->ramp_down)
    {
        Logger_Log(LOG_INFO, "[CFG][%s] Ramp up=%u down=%u",
                   MotorTag(motor),
                   (unsigned)cfg->ramp_up, (unsigned)cfg->ramp_down);
    }

    if (cfg->kick_duty || cfg->kick_ms)
    {
        const char *kick_str = (cfg->kick_enabled == 1) ? "ON" :
                               (cfg->kick_enabled == 0) ? "OFF" : "??";
        Logger_Log(LOG_INFO, "[CFG][%s] Kick %s duty=%u ms=%u",
                   MotorTag(motor), kick_str,
                   (unsigned)cfg->kick_duty, (unsigned)cfg->kick_ms);
    }

    if (cfg->telper)
    {
        Logger_Log(LOG_INFO, "[CFG][%s] TelPer=%u",
                   MotorTag(motor), (unsigned)cfg->telper);
    }
}

void MotorTuningConfig_PrintAll(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        MotorTuningConfig_Print((MotorId_t)i);
    }
}
