#include "operating_mode.h"
#include "activity_light.h"

/* ── Private state ────────────────────────────────────────────────────────── */
static RoverMode_t s_operatingMode = ROVER_MODE_DISARM;

/* ── Public functions ─────────────────────────────────────────────────────── */

void OperatingMode_Init(void)
{
    s_operatingMode = ROVER_MODE_DISARM;
    ActivityLight_SetMode(ROVER_MODE_DISARM);
}

void OperatingMode_Set(RoverMode_t mode)
{
    if (mode != ROVER_MODE_DISARM && mode != ROVER_MODE_MANUAL &&
        mode != ROVER_MODE_AUTONOMOUS)
    {
        mode = ROVER_MODE_DISARM; /* unknown -> safe state */
    }

    s_operatingMode = mode;
    ActivityLight_SetMode(mode);
}

RoverMode_t OperatingMode_Get(void)
{
    return s_operatingMode;
}

bool OperatingMode_IsDisarm(void)
{
    return (s_operatingMode == ROVER_MODE_DISARM);
}

const char *OperatingMode_ToString(RoverMode_t mode)
{
    switch (mode)
    {
        case ROVER_MODE_DISARM:     return "DISARM";
        case ROVER_MODE_MANUAL:     return "MANUAL";
        case ROVER_MODE_AUTONOMOUS: return "AUTONOMOUS";
        default:                    return "DISARM";
    }
}
