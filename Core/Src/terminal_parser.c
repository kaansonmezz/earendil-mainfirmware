#include "terminal_parser.h"
#include "activity_light.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define MAX_LINE_LEN 64

#define RPM_MAX   200
#define DUTY_MAX  255

static bool allDigits(const char *str)
{
    if (*str == '\0')
        return false;
    for (const char *p = str; *p; p++)
    {
        if (!isdigit((unsigned char)*p))
            return false;
    }
    return true;
}

/* Common setup for a motion command. Stores both the clamped and original
 * value plus clamping state. Does not execute or log anything. */
static void FillMotion(TerminalCommand_t *out, Direction_t dir,
                       int raw, int max)
{
    out->type          = TCMD_MOTION;
    out->motion.direction = dir;

    out->originalValue = (uint16_t)raw;
    out->hasValue      = true;

    if (raw > max)
    {
        out->value     = (uint16_t)max;
        out->wasClamped = true;
    }
    else
    {
        out->value     = (uint16_t)raw;
        out->wasClamped = false;
    }

    out->motion.speed  = (uint8_t)out->value;
}

bool TerminalParser_Parse(const char *line, TerminalCommand_t *outResult)
{
    if (line == NULL || outResult == NULL)
        return false;

    memset(outResult, 0, sizeof(*outResult));
    outResult->isDuty = false;

    char buf[MAX_LINE_LEN];
    size_t len = strlen(line);
    if (len >= MAX_LINE_LEN)
        return false;

    memcpy(buf, line, len + 1);

    while (*buf && isspace((unsigned char)buf[0]))
        memmove(buf, buf + 1, strlen(buf));

    len = strlen(buf);
    while (len > 0 && isspace((unsigned char)buf[len - 1]))
        buf[--len] = '\0';

    for (size_t i = 0; i < len; i++)
        buf[i] = (char)tolower((unsigned char)buf[i]);

    /* ── 1. help ─────────────────────────────────────────────────────── */
    if (strcmp(buf, "help") == 0)
    {
        outResult->type = TCMD_HELP;
        return true;
    }

    /* ── 2. stop ─────────────────────────────────────────────────────── */
    if (strcmp(buf, "stop") == 0)
    {
        outResult->type = TCMD_STOP;
        outResult->motion.direction = DIR_STOP;
        outResult->motion.speed = 0;
        return true;
    }

    /* ── 3. brake ────────────────────────────────────────────────────── */
    if (strcmp(buf, "brake") == 0)
    {
        outResult->type = TCMD_BRAKE;
        return true;
    }

    /* ── 4. identify ─────────────────────────────────────────────────── */
    if (strcmp(buf, "identify") == 0)
    {
        outResult->type = TCMD_IDENTIFY;
        return true;
    }

    /* ── 5. status ───────────────────────────────────────────────────── */
    if (strcmp(buf, "status") == 0)
    {
        outResult->type = TCMD_STATUS;
        return true;
    }

    /* ── 6. m rpm (control mode) ─────────────────────────────────────── */
    if (strcmp(buf, "m rpm") == 0)
    {
        outResult->type = TCMD_MODE_RPM;
        return true;
    }

    /* ── 7. m pwm (control mode) ─────────────────────────────────────── */
    if (strcmp(buf, "m pwm") == 0)
    {
        outResult->type = TCMD_MODE_PWM;
        return true;
    }

    /* ── 8. mode (plain) ─────────────────────────────────────────────── */
    if (strcmp(buf, "mode") == 0)
    {
        outResult->type = TCMD_MODE_QUERY;
        return true;
    }

    /* ── 9. drive mode────────────────────────────────────────────────── */
    if (strcmp(buf, "mode disarm") == 0)
    {
        ActivityLight_SetMode(ROVER_MODE_DISARM);
        outResult->type = TCMD_STOP;
        outResult->motion.direction = DIR_STOP;
        outResult->motion.speed = 0;
        return true;
    }

    if (strcmp(buf, "mode manual") == 0)
    {
        ActivityLight_SetMode(ROVER_MODE_MANUAL);
        outResult->type = TCMD_STOP;
        outResult->motion.direction = DIR_STOP;
        outResult->motion.speed = 0;
        return true;
    }

    if (strcmp(buf, "mode auto") == 0)
    {
        ActivityLight_SetMode(ROVER_MODE_AUTONOMOUS);
        outResult->type = TCMD_STOP;
        outResult->motion.direction = DIR_STOP;
        outResult->motion.speed = 0;
        return true;
    }

    /* ── 10. fd / bd / rd / ld (duty, value 0..255) ──────────────────── */
    if (len >= 3 && buf[1] == 'd' &&
        (buf[0] == 'f' || buf[0] == 'b' || buf[0] == 'r' || buf[0] == 'l'))
    {
        const char *valStr = buf + 2;
        if (!allDigits(valStr))
            return false;

        int val = atoi(valStr);
        outResult->isDuty = true;

        Direction_t dir = DIR_STOP;
        switch (buf[0])
        {
            case 'f': dir = DIR_FORWARD;  break;
            case 'b': dir = DIR_BACKWARD; break;
            case 'r': dir = DIR_RIGHT;    break;
            case 'l': dir = DIR_LEFT;     break;
        }

        FillMotion(outResult, dir, val, DUTY_MAX);
        return true;
    }

    /* ── 11. f / b / r / l (RPM, value 0..200) ──────────────────────── */
    if (len >= 2)
    {
        char dir = buf[0];
        if (dir == 'f' || dir == 'b' || dir == 'r' || dir == 'l')
        {
            const char *valStr = buf + 1;
            if (allDigits(valStr))
            {
                int val = atoi(valStr);

                Direction_t direction = DIR_STOP;
                switch (dir)
                {
                    case 'f': direction = DIR_FORWARD;  break;
                    case 'b': direction = DIR_BACKWARD; break;
                    case 'r': direction = DIR_RIGHT;    break;
                    case 'l': direction = DIR_LEFT;     break;
                }

                FillMotion(outResult, direction, val, RPM_MAX);
                return true;
            }
        }
    }

    return false;
}
