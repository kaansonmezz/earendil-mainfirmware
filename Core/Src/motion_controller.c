#include "motion_controller.h"
#include "motor_dispatcher.h"
#include "logger.h"
#include <string.h>

static MotorCmd_t motorCmds[MOTOR_COUNT];

static void SetMotor(MotorId_t id, MotorDir_t dir, uint16_t pwm)
{
    motorCmds[id].dir = dir;
    motorCmds[id].pwm = pwm;
}

static void SetAllMotors(MotorDir_t dir, uint16_t pwm)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        SetMotor((MotorId_t)i, dir, pwm);
}

static void SetMotorCmd(MotorId_t id, MotorDir_t dir, uint16_t pwm)
{
    motorCmds[id].dir = dir;
    motorCmds[id].pwm = pwm;
}

void MotionController_Init(void)
{
    memset(motorCmds, 0, sizeof(motorCmds));
}

/* Right-side motors (FR, RR) are physically mounted in reverse.
 * Invert their direction so logical forward/backward commands
 * produce the correct physical motion. */
static void ApplyMotorPolarity(MotorCmd_t cmds[MOTOR_COUNT])
{
    static const char *names[] = {"FL", "FR", "RL", "RR"};
    static const char *dirStr[] = {"STOP", "FWD", "BWD"};

    for (int i = 0; i < MOTOR_COUNT; i++)
    {
        if (i == MOTOR_FR || i == MOTOR_RR)
        {
            MotorDir_t orig = cmds[i].dir;
            if (orig == MCMD_FORWARD)
                cmds[i].dir = MCMD_BACKWARD;
            else if (orig == MCMD_BACKWARD)
                cmds[i].dir = MCMD_FORWARD;

            if (orig != cmds[i].dir)
                Logger_Log(LOG_INFO, "[POLARITY] %s: %s -> %s",
                           names[i], dirStr[orig], dirStr[cmds[i].dir]);
        }
    }
}

void MotionController_Execute(const MotionCmd_t *cmd)
{
    if (cmd == NULL)
        return;

    uint16_t spd = cmd->speed;

    switch (cmd->direction)
    {
        case DIR_FORWARD:
            SetAllMotors(MCMD_FORWARD, spd);
            break;

        case DIR_BACKWARD:
            SetAllMotors(MCMD_BACKWARD, spd);
            break;

        case DIR_LEFT:
            SetMotorCmd(MOTOR_FL, MCMD_BACKWARD, spd);
            SetMotorCmd(MOTOR_FR, MCMD_FORWARD, spd);
            SetMotorCmd(MOTOR_RL, MCMD_BACKWARD, spd);
            SetMotorCmd(MOTOR_RR, MCMD_FORWARD, spd);
            break;

        case DIR_RIGHT:
            SetMotorCmd(MOTOR_FL, MCMD_FORWARD, spd);
            SetMotorCmd(MOTOR_FR, MCMD_BACKWARD, spd);
            SetMotorCmd(MOTOR_RL, MCMD_FORWARD, spd);
            SetMotorCmd(MOTOR_RR, MCMD_BACKWARD, spd);
            break;

        case DIR_STOP:
        default:
            SetAllMotors(MCMD_STOP, 0);
            break;
    }

    ApplyMotorPolarity(motorCmds);
    Logger_Log(LOG_INFO, "Motion: dir=%d spd=%d", cmd->direction, cmd->speed);
    MotorDispatcher_SendAll(motorCmds);
}

void MotionController_Stop(void)
{
    SetAllMotors(MCMD_STOP, 0);
    MotorDispatcher_SendAll(motorCmds);
    Logger_Log(LOG_INFO, "Motion: STOP");
}

void MotionController_DisarmSafe(void)
{
    /* Neutralize any stale motion state so a command queued before DISARM
     * can never execute after leaving DISARM.  This zeroes the internal
     * command table without relying on a fresh command arriving. */
    SetAllMotors(MCMD_STOP, 0);
}
