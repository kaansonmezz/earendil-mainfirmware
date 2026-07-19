#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "rover_types.h"
#include "terminal_parser.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void MotionController_Init(void);
void MotionController_Execute(const MotionCmd_t *cmd);
void MotionController_Stop(void);
void MotionController_DisarmSafe(void);  /* stop + neutralize stale motion state */

/* Arc-turn drive: blended arc/tank-turn model with active brake point.
 * Uses signed inner-side computation: inner = target*(1000-2*tr)/1000.
 *   tr=0.00  — straight (both sides same direction at target)
 *   tr=0.50  — inner side BRAKE, outer at target
 *   tr=1.00  — full tank turn (opposite directions at target)
 * `isDuty`     — true for duty mode, false for RPM mode.
 * `target`     — outer-side speed (RPM 0..200 or duty 0..4000).
 * `motion`     — DRIVE_ARC_FL / FR / BL / BR.
 * `trPermille` — turn ratio 0..1000 (0=straight, 500=brake, 1000=pivot).
 * `outInner`   — receives the signed inner speed (negative = opposite dir).
 * `outOuter`   — receives the outer speed. */
void MotionController_ExecuteArcTurn(bool isDuty, uint16_t target,
                                     DriveArcMotion_t motion,
                                     uint16_t trPermille,
                                     int32_t *outInner,
                                     uint16_t *outOuter);

#endif /* MOTION_CONTROLLER_H */
