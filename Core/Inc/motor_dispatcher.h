#ifndef MOTOR_DISPATCHER_H
#define MOTOR_DISPATCHER_H

#include "rover_types.h"
#include "motor_link.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
void MotorDispatcher_Init(void);
void MotorDispatcher_Send(MotorId_t id, const MotorCmd_t *cmd);
void MotorDispatcher_SendAll(const MotorCmd_t cmds[MOTOR_COUNT]);
/* Send a raw string to all motor UARTs.  Returns true only if every motor
 * channel accepted the frame (immediate TX or successfully staged as pending).
 * Callers that need to know whether the dispatch actually reached the TX path
 * (e.g. the synchronized control-mode switch) must check this return value. */
bool MotorDispatcher_SendRaw(const char *msg);

/* Send a raw string to ONE motor UART only (no broadcast).  Routes the
 * frame through the same TX DMA busy/pending mechanism as SendRaw, so
 * the same safety/ordering guarantees apply.  Returns true if the frame
 * was accepted (immediate TX or successfully staged as pending).
 *
 * NOTE: unlike MotorDispatcher_SendRaw, this function intentionally
 * does NOT log the frame — callers (e.g. the direct-motor command
 * handler) own the [RAW][<tag>] <text> log line to avoid duplicates. */
bool MotorDispatcher_SendRawToMotor(MotorId_t motor, const char *msg);

MotorLink_t *MotorDispatcher_GetLink(MotorId_t id);
void MotorDispatcher_Update(void);

#endif /* MOTOR_DISPATCHER_H */
