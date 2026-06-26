#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "rover_types.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void MotionController_Init(void);
void MotionController_Execute(const MotionCmd_t *cmd);
void MotionController_Stop(void);
void MotionController_DisarmSafe(void);  /* stop + neutralize stale motion state */

#endif /* MOTION_CONTROLLER_H */
