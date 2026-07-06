#ifndef MOTOR_TUNING_CONFIG_H
#define MOTOR_TUNING_CONFIG_H

#include "rover_types.h"

/* ── Per-motor tuning config cache ──────────────────────────────────────── */
#define TUNING_SLOTS 8
#define CFG_ERROR_MAX 32

typedef struct
{
    uint8_t  valid;          /* 1 when all three parts received and committed */
    uint8_t  has_pi;
    uint8_t  has_base;
    uint8_t  has_boost;

    int32_t  kp_m;           /* raw fixed-point (value * 1000) */
    int32_t  ki_m;           /* raw fixed-point (value * 1000) */

    uint16_t base_pwm[TUNING_SLOTS];
    uint16_t boost_pwm[TUNING_SLOTS];
    uint16_t boost_ms;

    /* Optional fields — may be 0 if F411 does not report them */
    uint16_t ramp_up;
    uint16_t ramp_down;
    uint16_t kick_duty;
    uint16_t kick_ms;
    uint8_t  kick_enabled;   /* 0 = OFF, 1 = ON, 0xFF = unknown */
    uint16_t telper;

    uint32_t update_count;
    uint32_t last_update_ms;

    char     last_error[CFG_ERROR_MAX]; /* last error string, e.g. "unsupported cfg" */
} MotorTuningConfig_t;

/* ── Public API ─────────────────────────────────────────────────────────── */
void MotorTuningConfig_Init(void);

/* Process a single motor UART RX line.  `motor` is the MotorId_t that
 * produced the line.  `line` is the raw payload (no [UARTx_RX] prefix). */
void MotorTuningConfig_ProcessLine(MotorId_t motor, const char *line);

/* Get read-only pointer to the cached config for a motor. */
const MotorTuningConfig_t *MotorTuningConfig_Get(MotorId_t motor);

/* Print cached config for one motor or all motors to the terminal. */
void MotorTuningConfig_Print(MotorId_t motor);
void MotorTuningConfig_PrintAll(void);

/* Map a UART RX slot index (as used in motor_uart_dma.c) to MotorId_t.
 * Slot ordering: 0=USART2(FL), 1=UART4(FR), 2=UART5(RR), 3=UART7(RL). */
MotorId_t MotorTuningConfig_SlotToMotorId(int slot);

#endif /* MOTOR_TUNING_CONFIG_H */
