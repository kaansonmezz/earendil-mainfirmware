#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "rover_types.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void SafetyManager_Init(void);
void SafetyManager_Update(void);
void SafetyManager_NotifyRx(MotorId_t id);
bool SafetyManager_IsLinkLost(MotorId_t id);

/* ── DISARM safety lock helpers ──────────────────────────────────────────── */
void SafetyManager_EnterDisarm(void);  /* safe-zero all motors + drop stale TX */
void SafetyManager_LeaveDisarm(void);  /* keep motors stopped, clear stale state */

#endif /* SAFETY_MANAGER_H */
