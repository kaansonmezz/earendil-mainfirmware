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

/* ── PC/Pi control-link watchdog ─────────────────────────────────────────── */
void SafetyManager_NotifyPcActivity(void);  /* refresh PC-link timestamp     */
bool SafetyManager_IsPcLinkAlive(void);     /* true if link within timeout   */
bool SafetyManager_IsPcLinkSeen(void);      /* true if any activity received */
bool SafetyManager_IsPcLinkTimeout(void);   /* true if timeout is latched    */
uint32_t SafetyManager_PcLinkAgeMs(void);   /* ms since last PC activity     */

#endif /* SAFETY_MANAGER_H */
