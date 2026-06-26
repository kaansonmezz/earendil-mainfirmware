#include "command_handler.h"
#include "control_mode.h"
#include "motion_controller.h"
#include "motor_dispatcher.h"
#include "activity_light.h"
#include "operating_mode.h"
#include "safety_manager.h"
#include "logger.h"

/* ── Helpers ──────────────────────────────────────────────────────────────── */

/* Returns the short terminal prefix for a motion command ("f", "fd", ...). */
static const char *MotionPrefix(Direction_t dir, bool isDuty)
{
    switch (dir)
    {
        case DIR_FORWARD:  return isDuty ? "fd" : "f";
        case DIR_BACKWARD: return isDuty ? "bd" : "b";
        case DIR_RIGHT:    return isDuty ? "rd" : "r";
        case DIR_LEFT:     return isDuty ? "ld" : "l";
        default:           return "?";
    }
}

/* ── Command classification helpers ─────────────────────────────────────────
 *  Used by the DISARM gate to decide what is allowed while locked. */

bool Command_IsModeTransition(const TerminalCommand_t *cmd)
{
    return (cmd != NULL && cmd->type == TCMD_OP_MODE);
}

bool Command_IsMotionCommand(const TerminalCommand_t *cmd)
{
    return (cmd != NULL && cmd->type == TCMD_MOTION);
}

/* ── Operating-mode transition ──────────────────────────────────────────────
 *  Enforces the DISARM safety lock side-effects: on entering DISARM all
 *  motors are safe-zeroed and stale motion is neutralized; on leaving,
 *  motors stay stopped and a fresh command is required to move. */
static void HandleOperatingMode(RoverMode_t target)
{
    if (target == ROVER_MODE_DISARM)
    {
        OperatingMode_Set(ROVER_MODE_DISARM);
        SafetyManager_EnterDisarm();
        Logger_Log(LOG_INFO, "[MODE] DISARM active, motion commands locked");
        return;
    }

    /* Leaving DISARM -> MANUAL or AUTONOMOUS */
    OperatingMode_Set(target);
    SafetyManager_LeaveDisarm();

    if (target == ROVER_MODE_MANUAL)
        Logger_Log(LOG_INFO, "[MODE] MANUAL active");
    else if (target == ROVER_MODE_AUTONOMOUS)
        Logger_Log(LOG_INFO, "[MODE] AUTONOMOUS active");
    else
        Logger_Log(LOG_INFO, "[MODE] %s active", OperatingMode_ToString(target));

    Logger_Log(LOG_INFO, "Motors stopped; send a motion command to move");
}

/* ── Public functions ─────────────────────────────────────────────────────── */

void CommandHandler_PrintHelp(void)
{
    Logger_Log(LOG_INFO, "Available commands:");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Control mode:");
    Logger_Log(LOG_INFO, "  m speed         Set RPM mode and forward \"mode speed\"");
    Logger_Log(LOG_INFO, "  m duty          Set PWM/duty mode and forward \"mode duty\"");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Rover mode (operating mode):");
    Logger_Log(LOG_INFO, "  mode             Show current rover mode (disarm/manual/auto)");
    Logger_Log(LOG_INFO, "  mode disarm      Disarm rover (red LED, STOP motors, lock motion)");
    Logger_Log(LOG_INFO, "  mode manual      Manual mode (green LED, motors stopped)");
    Logger_Log(LOG_INFO, "  mode auto        Autonomous mode (yellow LED, motors stopped)");
    Logger_Log(LOG_INFO, "  mode autonomous  Alias for 'mode auto'");
    Logger_Log(LOG_INFO, "  While DISARM: only mode/status/help/stop/brake are accepted.");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "RPM mode commands:");
    Logger_Log(LOG_INFO, "  f0..f200         Forward RPM command");
    Logger_Log(LOG_INFO, "  b0..b200         Backward RPM command");
    Logger_Log(LOG_INFO, "  r0..r200         Right turn RPM command");
    Logger_Log(LOG_INFO, "  l0..l200         Left turn RPM command");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "PWM mode commands:");
    Logger_Log(LOG_INFO, "  fd0..fd255       Forward PWM/duty command");
    Logger_Log(LOG_INFO, "  bd0..bd255       Backward PWM/duty command");
    Logger_Log(LOG_INFO, "  rd0..rd255       Right turn PWM/duty command");
    Logger_Log(LOG_INFO, "  ld0..ld255       Left turn PWM/duty command");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Common commands:");
    Logger_Log(LOG_INFO, "  stop             Stop motors");
    Logger_Log(LOG_INFO, "  brake            Send brake command: x");
    Logger_Log(LOG_INFO, "  identify         Send identify to all motor UARTs");
    Logger_Log(LOG_INFO, "  status           Send status to all motor UARTs");
    Logger_Log(LOG_INFO, "  help             Show this command list");
}

void CommandHandler_Handle(const TerminalCommand_t *cmd)
{
    if (cmd == NULL)
        return;

    /* ── Primary DISARM safety gate ────────────────────────────────────
     * While DISARM is active, only mode transitions and harmless query/
     * stop/brake commands are accepted.  Every motion-causing or
     * control-changing command is rejected so the rover cannot move. */
    if (OperatingMode_IsDisarm())
    {
        bool allowed = false;
        switch (cmd->type)
        {
            case TCMD_OP_MODE:      /* mode disarm/manual/auto/autonomous */
            case TCMD_HELP:         /* help */
            case TCMD_STATUS:       /* status (query) */
            case TCMD_MODE_QUERY:   /* mode (query) */
            case TCMD_STOP:         /* stop (safe) */
            case TCMD_BRAKE:        /* brake (safe) */
                allowed = true;
                break;

            default:
                allowed = false;
                break;
        }

        if (!allowed)
        {
            Logger_Log(LOG_WARN, "[DISARM] Command ignored. Change mode first.");
            return;
        }
    }

    switch (cmd->type)
    {
        case TCMD_HELP:
            CommandHandler_PrintHelp();
            break;

        case TCMD_OP_MODE:
            HandleOperatingMode(cmd->opMode);
            break;

        case TCMD_STOP:
            MotionController_Execute(&cmd->motion);
            break;

        case TCMD_MOTION:
        {
            /* Report clamping first (was previously emitted by the parser). */
            if (cmd->wasClamped)
            {
                Logger_Log(LOG_WARN, "%s value %u clamped to %u",
                           MotionPrefix(cmd->motion.direction, cmd->isDuty),
                           cmd->originalValue, cmd->value);
            }

            ControlMode_t mode = ControlMode_Get();

            if (cmd->isDuty && mode != CONTROL_MODE_PWM)
            {
                Logger_Log(LOG_ERROR, "Invalid mode: duty commands require PWM mode");
                break;
            }

            if (!cmd->isDuty && mode != CONTROL_MODE_RPM)
            {
                Logger_Log(LOG_ERROR, "Invalid mode: RPM commands require RPM mode");
                break;
            }

            MotionController_Execute(&cmd->motion);
            break;
        }

        case TCMD_BRAKE:
            MotorDispatcher_SendRaw("x");
            break;

        case TCMD_MODE_RPM:
            ControlMode_Set(CONTROL_MODE_RPM);
            Logger_Log(LOG_INFO, "Control mode set to RPM");
            MotorDispatcher_SendRaw("mode speed");
            break;

        case TCMD_MODE_PWM:
            ControlMode_Set(CONTROL_MODE_PWM);
            Logger_Log(LOG_INFO, "Control mode set to PWM");
            MotorDispatcher_SendRaw("mode duty");
            break;

        case TCMD_MODE_QUERY:
            Logger_Log(LOG_INFO, "Rover mode: %s",
                       OperatingMode_ToString(OperatingMode_Get()));
            break;

        case TCMD_IDENTIFY:
            MotorDispatcher_SendRaw("identify");
            break;

        case TCMD_STATUS:
            MotorDispatcher_SendRaw("status");
            break;

        default:
            break;
    }
}
