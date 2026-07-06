#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "rover_types.h"
#include "terminal_parser.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void MotionController_Init(void);
void MotionController_Execute(const MotionCmd_t *cmd);
void MotionController_Stop(void);
void MotionController_DisarmSafe(void);  /* stop + neutralize stale motion state */

/* Arc-turn drive: compute inner/outer speeds from turn ratio and dispatch
 * to motors via the same polarity-corrected path as normal motion commands.
 * `isDuty`  — true for duty mode, false for RPM mode.
 * `target`  — outer-side speed (RPM 0..200 or duty 0..4000).
 * `motion`  — DRIVE_ARC_FL / FR / BL / BR.
 * `trPermille` — turn ratio 0..1000 (0=straight, 1000=pivot).
 * `outInner` / `outOuter` — receives the computed inner/outer speeds. */
void MotionController_ExecuteArcTurn(bool isDuty, uint16_t target,
                                     DriveArcMotion_t motion,
                                     uint16_t trPermille,
                                     uint16_t *outInner,
                                     uint16_t *outOuter);

#endif /* MOTION_CONTROLLER_H */
