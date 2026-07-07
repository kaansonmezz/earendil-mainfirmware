#ifndef ROVER_TYPES_H
#define ROVER_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* ── Motor identifiers ──────────────────────────────────────────────────── */
typedef enum
{
    MOTOR_FL = 0,   /* Front Left  – USART2 */
    MOTOR_FR,       /* Front Right – UART4  */
    MOTOR_RL,       /* Rear Left   – UART7  */
    MOTOR_RR,       /* Rear Right  – UART5  */
    MOTOR_COUNT
} MotorId_t;

/* ── Direction commands ──────────────────────────────────────────────────── */
typedef enum
{
    DIR_FORWARD = 0,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT,
    DIR_STOP
} Direction_t;

/* ── Motor-level direction ──────────────────────────────────────────────── */
typedef enum
{
    MCMD_STOP = 0,
    MCMD_FORWARD,
    MCMD_BACKWARD,
    MCMD_BRAKE           /* active brake ("x" to F411) */
} MotorDir_t;

/* ── Motor command (direction + PWM per wheel) ──────────────────────────── */
typedef struct
{
    MotorDir_t dir;
    uint16_t   pwm;      /* 0–4000 */
} MotorCmd_t;

/* ── Motion command (abstract, from terminal) ───────────────────────────── */
typedef struct
{
    Direction_t direction;
    uint16_t    speed;   /* 0–4000 */
} MotionCmd_t;

/* ── Control mode (RPM / PWM) ─────────────────────────────────────────────── */
/* Defined in control_mode.h */

/* ── ACK status ─────────────────────────────────────────────────────────── */
typedef enum
{
    ACK_NONE = 0,
    ACK_OK,
    ACK_TIMEOUT,
    ACK_ERROR
} AckStatus_t;

/* ── Motor link state (per motor UART) ──────────────────────────────────── */
typedef enum
{
    LINK_IDLE = 0,
    LINK_WAIT_ACK,
    LINK_ACKED,
    LINK_TIMEOUT
} LinkState_t;

/* ── Rover operating mode (DISARM / MANUAL / AUTONOMOUS) ────────────────── */
/*   Distinct from the RPM/PWM control mode.  Authority lives in
 *   operating_mode.c; activity_light.c only drives the GPIO LEDs. */
typedef enum
{
    ROVER_MODE_DISARM = 0,
    ROVER_MODE_MANUAL,
    ROVER_MODE_AUTONOMOUS
} RoverMode_t;

#endif /* ROVER_TYPES_H */
