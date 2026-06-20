#ifndef MOTOR_LINK_H
#define MOTOR_LINK_H

#include "app_config.h"

/* ── MotorLink context ──────────────────────────────────────────────────── */
typedef struct
{
    MotorId_t   id;
    LinkState_t state;
    uint32_t    lastTxTick;
    uint8_t     retryCount;
} MotorLink_t;

/* ── Public API ─────────────────────────────────────────────────────────── */
void MotorLink_Init(MotorLink_t *link, MotorId_t id);
void MotorLink_Update(MotorLink_t *link);

#endif /* MOTOR_LINK_H */
