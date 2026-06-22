#include "command_handler.h"
#include "control_mode.h"
#include "motion_controller.h"
#include "motor_dispatcher.h"
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

/* ── Public functions ─────────────────────────────────────────────────────── */

void CommandHandler_PrintHelp(void)
{
    Logger_Log(LOG_INFO, "Available commands:");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Modes:");
    Logger_Log(LOG_INFO, "  mode             Show current mode");
    Logger_Log(LOG_INFO, "  mode rpm         Set RPM mode and forward \"mode rpm\"");
    Logger_Log(LOG_INFO, "  mode pwm         Set PWM mode and forward \"mode pwm\"");
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

    switch (cmd->type)
    {
        case TCMD_HELP:
            CommandHandler_PrintHelp();
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
            MotorDispatcher_SendRaw("mode rpm");
            break;

        case TCMD_MODE_PWM:
            ControlMode_Set(CONTROL_MODE_PWM);
            Logger_Log(LOG_INFO, "Control mode set to PWM");
            MotorDispatcher_SendRaw("mode pwm");
            break;

        case TCMD_MODE_QUERY:
            Logger_Log(LOG_INFO, "Current mode: %s",
                       ControlMode_ToString(ControlMode_Get()));
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
