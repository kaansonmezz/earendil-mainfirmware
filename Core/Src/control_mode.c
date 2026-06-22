#include "control_mode.h"

/* ── Private state ────────────────────────────────────────────────────────── */
static ControlMode_t s_controlMode = CONTROL_MODE_RPM;

/* ── Public functions ─────────────────────────────────────────────────────── */

void ControlMode_Init(void)
{
    s_controlMode = CONTROL_MODE_RPM;
}

void ControlMode_Set(ControlMode_t mode)
{
    s_controlMode = mode;
}

ControlMode_t ControlMode_Get(void)
{
    return s_controlMode;
}

const char *ControlMode_ToString(ControlMode_t mode)
{
    return (mode == CONTROL_MODE_RPM) ? "RPM" : "PWM";
}
