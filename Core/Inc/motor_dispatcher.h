#ifndef MOTOR_DISPATCHER_H
#define MOTOR_DISPATCHER_H

#include "rover_types.h"
#include "motor_link.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void MotorDispatcher_Init(void);
void MotorDispatcher_Send(MotorId_t id, const MotorCmd_t *cmd);
void MotorDispatcher_SendAll(const MotorCmd_t cmds[MOTOR_COUNT]);
void MotorDispatcher_SendRaw(const char *msg);
MotorLink_t *MotorDispatcher_GetLink(MotorId_t id);
void MotorDispatcher_Update(void);

#endif /* MOTOR_DISPATCHER_H */
