#ifndef ACK_MANAGER_H
#define ACK_MANAGER_H

#include "rover_types.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void        AckManager_Init(void);
void        AckManager_RegisterPending(MotorId_t id);
AckStatus_t AckManager_CheckStatus(MotorId_t id);
void        AckManager_Update(void);

#endif /* ACK_MANAGER_H */
