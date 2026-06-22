#ifndef CONTROL_MODE_H
#define CONTROL_MODE_H

#include <stdint.h>

/* ── Control mode (RPM / PWM) ─────────────────────────────────────────────── */
typedef enum
{
    CONTROL_MODE_RPM = 0,
    CONTROL_MODE_PWM
} ControlMode_t;

/* ── Public API ───────────────────────────────────────────────────────────── */
void          ControlMode_Init(void);
void          ControlMode_Set(ControlMode_t mode);
ControlMode_t ControlMode_Get(void);
const char   *ControlMode_ToString(ControlMode_t mode);

#endif /* CONTROL_MODE_H */
