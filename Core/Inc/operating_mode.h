#ifndef OPERATING_MODE_H
#define OPERATING_MODE_H

#include "rover_types.h"

/* ── Rover operating mode authority ────────────────────────────────────────
 *  Single source of truth for the DISARM / MANUAL / AUTONOMOUS operating
 *  mode.  This is the safety lock used by command_handler.c and
 *  motor_dispatcher.c to gate motion while DISARM is active.
 *
 *  activity_light.c only drives the GPIO LEDs; OperatingMode_Set() calls
 *  ActivityLight_SetMode() so the LED state always tracks this module.
 * ─────────────────────────────────────────────────────────────────────────── */

void         OperatingMode_Init(void);
void         OperatingMode_Set(RoverMode_t mode);
RoverMode_t  OperatingMode_Get(void);
bool         OperatingMode_IsDisarm(void);
const char  *OperatingMode_ToString(RoverMode_t mode);

#endif /* OPERATING_MODE_H */
