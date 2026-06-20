#ifndef SAFETY_MANAGER_H
#define SAFETY_MANAGER_H

#include "rover_types.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void SafetyManager_Init(void);
void SafetyManager_Update(void);
void SafetyManager_NotifyRx(MotorId_t id);
bool SafetyManager_IsLinkLost(MotorId_t id);

#endif /* SAFETY_MANAGER_H */
