#include "motion_controller.h"
#include "motor_dispatcher.h"
#include "control_mode.h"
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

void MotionController_ExecuteArcTurn(bool isDuty, uint16_t target,
                                     DriveArcMotion_t motion,
                                     uint16_t trPermille,
                                     uint16_t *outInner,
                                     uint16_t *outOuter)
{
    /* Control mode gate */
    ControlMode_t mode = ControlMode_Get();
    if (isDuty && mode != CONTROL_MODE_PWM)
    {
        Logger_Log(LOG_ERROR,
                   "[DRIVE] Control mode mismatch: DUTY command while RPM mode active");
        return;
    }
    if (!isDuty && mode != CONTROL_MODE_RPM)
    {
        Logger_Log(LOG_ERROR,
                   "[DRIVE] Control mode mismatch: RPM command while DUTY mode active");
        return;
    }

    /* Compute inner/outer speeds */
    uint16_t inner = (uint16_t)((uint32_t)target * (1000 - trPermille) / 1000);
    uint16_t outer = target;

    if (outInner) *outInner = inner;
    if (outOuter) *outOuter = outer;

    /* Map arc motion to per-motor logical commands (before polarity correction).
     * Forward arcs:  left-side = inner, right-side = outer, both MCMD_FORWARD.
     * Backward arcs: left-side = inner, right-side = outer, both MCMD_BACKWARD.
     *
     * After ApplyMotorPolarity(), FR/RR (physically reversed) get their
     * direction flipped, producing the correct physical motion. */
    MotorDir_t dir;
    uint16_t   leftSpd, rightSpd;

    switch (motion)
    {
        case DRIVE_ARC_FL:
            dir     = MCMD_FORWARD;
            leftSpd = inner;
            rightSpd = outer;
            break;
        case DRIVE_ARC_FR:
            dir     = MCMD_FORWARD;
            leftSpd = outer;
            rightSpd = inner;
            break;
        case DRIVE_ARC_BL:
            dir     = MCMD_BACKWARD;
            leftSpd = inner;
            rightSpd = outer;
            break;
        case DRIVE_ARC_BR:
            dir     = MCMD_BACKWARD;
            leftSpd = outer;
            rightSpd = inner;
            break;
        default:
            return;
    }

    SetMotorCmd(MOTOR_FL, dir, leftSpd);
    SetMotorCmd(MOTOR_RL, dir, leftSpd);
    SetMotorCmd(MOTOR_FR, dir, rightSpd);
    SetMotorCmd(MOTOR_RR, dir, rightSpd);

    ApplyMotorPolarity(motorCmds);

    const char *modeStr = isDuty ? "DUTY" : "RPM";
    const char *motionName;
    switch (motion)
    {
        case DRIVE_ARC_FL: motionName = "FL"; break;
        case DRIVE_ARC_FR: motionName = "FR"; break;
        case DRIVE_ARC_BL: motionName = "BL"; break;
        case DRIVE_ARC_BR: motionName = "BR"; break;
        default:            motionName = "??"; break;
    }

    Logger_Log(LOG_INFO,
               "[DRIVE] mode=%s target=%u motion=%s tr=%u.%02u tr_permille=%u inner=%u outer=%u",
               modeStr, target, motionName,
               trPermille / 1000, (trPermille % 1000) / 10,
               trPermille, inner, outer);

    if (dir == MCMD_BACKWARD)
        Logger_Log(LOG_INFO, "[DRIVE_LOGICAL] left=-%u right=-%u", leftSpd, rightSpd);
    else
        Logger_Log(LOG_INFO, "[DRIVE_LOGICAL] left=+%u right=+%u", leftSpd, rightSpd);

    MotorDispatcher_SendAll(motorCmds);
}
