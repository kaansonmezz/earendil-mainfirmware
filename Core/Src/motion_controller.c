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
    static const char *dirStr[] = {"STOP", "FWD", "BWD", "BRK"};

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
                                     int32_t *outInner,
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

    /* When target is zero, STOP all motors (coast), not BRAKE. */
    if (target == 0)
    {
        SetAllMotors(MCMD_STOP, 0);
        ApplyMotorPolarity(motorCmds);
        Logger_Log(LOG_INFO, "[DRIVE] target=0 -> STOP all motors");
        MotorDispatcher_SendAll(motorCmds);
        if (outInner) *outInner = 0;
        if (outOuter) *outOuter = 0;
        return;
    }

    /* ── Signed blended arc/tank-turn model ──────────────────────────
     *
     *   signedInner = target * (1000 - 2 * trPermille) / 1000
     *
     *   tr=0.00 → signedInner = +target  (straight)
     *   tr=0.25 → signedInner = +target/2 (arc, same direction)
     *   tr=0.50 → signedInner =  0        (inner BRAKE)
     *   tr=0.75 → signedInner = -target/2 (arc, opposite direction)
     *   tr=1.00 → signedInner = -target   (full tank turn)
     *
     * BRAKE is only issued at the exact brake point (trPermille == 500).
     * Anti-rounding prevents integer truncation from accidentally producing
     * zero near the brake point. */
    int32_t signedInner = (int32_t)target * (1000 - 2 * (int32_t)trPermille) / 1000;
    uint16_t outer = target;

    /* Anti-rounding: clamp to at least ±1 when not at the exact brake point. */
    if (trPermille < 500 && signedInner == 0)
        signedInner = 1;
    else if (trPermille > 500 && signedInner == 0)
        signedInner = -1;

    if (outInner) *outInner = signedInner;
    if (outOuter) *outOuter = outer;

    /* ── Map to per-motor logical commands (before polarity correction).
     *
     * Outer side: always moves in the requested direction (outerDir).
     * Inner side:
     *   signedInner > 0 → same direction as outer
     *   signedInner == 0 → BRAKE (only at trPermille == 500)
     *   signedInner < 0 → opposite direction to outer
     *
     * After ApplyMotorPolarity(), FR/RR (physically reversed) get their
     * direction flipped, producing the correct physical motion. */
    MotorDir_t outerDir;
    bool       leftIsInner;

    switch (motion)
    {
        case DRIVE_ARC_FL: outerDir = MCMD_FORWARD;  leftIsInner = true;  break;
        case DRIVE_ARC_FR: outerDir = MCMD_FORWARD;  leftIsInner = false; break;
        case DRIVE_ARC_BL: outerDir = MCMD_BACKWARD; leftIsInner = true;  break;
        case DRIVE_ARC_BR: outerDir = MCMD_BACKWARD; leftIsInner = false; break;
        default: return;
    }

    /* Determine inner-side direction and magnitude. */
    MotorDir_t innerDir;
    uint16_t   innerMag;

    if (trPermille == 500)
    {
        /* Exact brake point: active brake on inner side. */
        innerDir = MCMD_BRAKE;
        innerMag = 0;
    }
    else if (signedInner > 0)
    {
        /* Same direction as outer. */
        innerDir = outerDir;
        innerMag = (uint16_t)signedInner;
    }
    else
    {
        /* Opposite direction to outer (signedInner < 0). */
        innerDir = (outerDir == MCMD_FORWARD) ? MCMD_BACKWARD : MCMD_FORWARD;
        innerMag = (uint16_t)(-signedInner);
    }

    /* Assign to left/right motor groups. */
    MotorDir_t leftDir, rightDir;
    uint16_t   leftSpd, rightSpd;

    if (leftIsInner)
    {
        leftDir  = innerDir;
        leftSpd  = innerMag;
        rightDir = outerDir;
        rightSpd = outer;
    }
    else
    {
        leftDir  = outerDir;
        leftSpd  = outer;
        rightDir = innerDir;
        rightSpd = innerMag;
    }

    SetMotorCmd(MOTOR_FL, leftDir,  leftSpd);
    SetMotorCmd(MOTOR_RL, leftDir,  leftSpd);
    SetMotorCmd(MOTOR_FR, rightDir, rightSpd);
    SetMotorCmd(MOTOR_RR, rightDir, rightSpd);

    ApplyMotorPolarity(motorCmds);

    /* ── Enhanced arc-turn logging ─────────────────────────────────── */
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

    static const char *dirNames[] = {"STOP", "FWD", "BWD", "BRK"};

    Logger_Log(LOG_INFO,
               "[DRIVE] mode=%s target=%u motion=%s tr=%u.%02u tr_permille=%u",
               modeStr, target, motionName,
               trPermille / 1000, (trPermille % 1000) / 10,
               trPermille);

    Logger_Log(LOG_INFO,
               "[DRIVE] inner_signed=%ld inner_dir=%s inner_mag=%u outer=%u outer_dir=%s",
               (long)signedInner, dirNames[innerDir], innerMag,
               outer, dirNames[outerDir]);

    Logger_Log(LOG_INFO, "[DRIVE_LOGICAL] left=%u(%s) right=%u(%s)",
               leftSpd, dirNames[leftDir], rightSpd, dirNames[rightDir]);

    MotorDispatcher_SendAll(motorCmds);
}
