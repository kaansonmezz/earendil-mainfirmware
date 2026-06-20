#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#include "rover_types.h"

/* ── Public API ─────────────────────────────────────────────────────────── */
uint16_t MotorProtocol_Encode(const MotorCmd_t *cmd, char *buf, uint16_t bufSize);

#endif /* MOTOR_PROTOCOL_H */
