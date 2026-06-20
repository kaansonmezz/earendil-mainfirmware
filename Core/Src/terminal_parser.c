#include "terminal_parser.h"
#include "logger.h"
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

bool TerminalParser_Parse(const char *line, TerminalResult_t *outResult)
{
    if (line == NULL || outResult == NULL)
        return false;

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

    /* ── 6. mode rpm ─────────────────────────────────────────────────── */
    if (strcmp(buf, "mode rpm") == 0)
    {
        outResult->type = TCMD_MODE_RPM;
        return true;
    }

    /* ── 7. mode pwm ─────────────────────────────────────────────────── */
    if (strcmp(buf, "mode pwm") == 0)
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

    /* ── 9. fd / bd / rd / ld (duty, value 0..255) ──────────────────── */
    if (len >= 3 && buf[1] == 'd' &&
        (buf[0] == 'f' || buf[0] == 'b' || buf[0] == 'r' || buf[0] == 'l'))
    {
        const char *valStr = buf + 2;
        if (!allDigits(valStr))
            return false;

        int val = atoi(valStr);
        if (val > DUTY_MAX)
        {
            Logger_Log(LOG_WARN, "%c%c value %d clamped to %d", buf[0], buf[1], val, DUTY_MAX);
            val = DUTY_MAX;
        }

        outResult->type = TCMD_MOTION;
        outResult->isDuty = true;
        outResult->motion.speed = (uint8_t)val;

        switch (buf[0])
        {
            case 'f': outResult->motion.direction = DIR_FORWARD;  break;
            case 'b': outResult->motion.direction = DIR_BACKWARD; break;
            case 'r': outResult->motion.direction = DIR_RIGHT;    break;
            case 'l': outResult->motion.direction = DIR_LEFT;     break;
        }
        return true;
    }

    /* ── 10. f / b / r / l (RPM, value 0..200) ──────────────────────── */
    if (len >= 2)
    {
        char dir = buf[0];
        if (dir == 'f' || dir == 'b' || dir == 'r' || dir == 'l')
        {
            const char *valStr = buf + 1;
            if (allDigits(valStr))
            {
                int val = atoi(valStr);
                if (val > RPM_MAX)
                {
                    Logger_Log(LOG_WARN, "%c value %d clamped to %d", dir, val, RPM_MAX);
                    val = RPM_MAX;
                }

                outResult->type = TCMD_MOTION;
                outResult->isDuty = false;
                outResult->motion.speed = (uint8_t)val;

                switch (dir)
                {
                    case 'f': outResult->motion.direction = DIR_FORWARD;  break;
                    case 'b': outResult->motion.direction = DIR_BACKWARD; break;
                    case 'r': outResult->motion.direction = DIR_RIGHT;    break;
                    case 'l': outResult->motion.direction = DIR_LEFT;     break;
                }
                return true;
            }
        }
    }

    return false;
}
