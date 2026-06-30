#include "command_handler.h"
#include "control_mode.h"
#include "motion_controller.h"
#include "motor_dispatcher.h"
#include "motor_tx_dma.h"
#include "activity_light.h"
#include "operating_mode.h"
#include "safety_manager.h"
#include "terminal_if.h"
#include "logger.h"
#include <string.h>

/* ── Tunables ───────────────────────────────────────────────────────────────
 *  Bounded wait for the motor TX DMA path to drain during a synchronized
 *  control-mode switch.  This is a main-loop context call (never ISR), so a
 *  short blocking poll on the TX busy/pending flags is safe and bounded. */
#define MODE_SWITCH_TX_DRAIN_MS 100U

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

/* Direct-motor raw payloads that are safe in DISARM (queries / stop /
 * brake / the documented control-mode switches).  Used by the DISARM
 * gate to reject motion-causing raw payloads like `FL f100` while still
 * allowing `FL status`, `FL identify`, `FL stop`, `FL x`,
 * `FL mode speed`, `FL mode duty`.  The payload reaches here already
 * lowercased and trimmed by the parser, so a plain strcmp is enough. */
static bool IsSafeRawPayload(const char *p)
{
    if (p == NULL)
        return false;
    return (strcmp(p, "status")     == 0 ||
            strcmp(p, "identify")  == 0 ||
            strcmp(p, "stop")      == 0 ||
            strcmp(p, "x")         == 0 ||
            strcmp(p, "mode speed")== 0 ||
            strcmp(p, "mode duty") == 0);
}

static const char *MotorTagName(MotorId_t id)
{
    switch (id)
    {
        case MOTOR_FL: return "FL";
        case MOTOR_FR: return "FR";
        case MOTOR_RL: return "RL";
        case MOTOR_RR: return "RR";
        default:        return "??";
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

/* ── Synchronized control-mode switch (RPM <-> PWM) ──────────────────────────
 *  The F411 motor controllers reject `mode speed` / `mode duty` while a motor
 *  is still running, replying e.g. "[ERR] Stop motor first".  That would leave
 *  H7 and F411 desynchronized (H7 believes it changed mode, F411 did not).
 *
 *  Safe policy implemented here:
 *    1. Announce the switch.
 *    2. Stop all motors first (`stop`), dropping any queued motion frame so
 *       the stop leaves immediately instead of being stalled behind a pending
 *       motion frame.
 *    3. Wait for the stop frame to fully drain on every motor UART (bounded
 *       poll on the TX DMA busy/pending flags).
 *    4. Send the requested `mode speed` / `mode duty` to all controllers.
 *    5. Only after every channel accepted the frame for TX dispatch, update
 *       the local control mode.
 *
 *  ACK confirmation from the F411 is NOT used: the existing ACK/OK parsing is
 *  not wired to raw commands (SendRaw does not register a pending ACK and the
 *  RX callback only logs replies), so we must not fake full confirmation.  The
 *  local mode is therefore advanced only after successful TX dispatch of the
 *  mode command, and the log explicitly states the change was dispatched but
 *  not fully ACK-confirmed. */
static bool WaitForTxDrain(uint32_t timeoutMs)
{
    uint32_t start = HAL_GetTick();
    while (!MotorTxDma_AllIdle())
    {
        if ((HAL_GetTick() - start) >= timeoutMs)
            return false;
    }
    return true;
}

static void HandleControlModeSwitch(ControlMode_t target)
{
    const char *modeName = (target == CONTROL_MODE_RPM) ? "SPEED" : "DUTY";
    const char *modeCmd  = (target == CONTROL_MODE_RPM) ? "mode speed" : "mode duty";

    Logger_Log(LOG_INFO, "[MODE] Switching motor controllers to %s...", modeName);

    /* 1. Stop all motors first so the F411s accept the mode command.
     *    Cancel any queued (non-active) motion frame so `stop` leaves
     *    immediately rather than sitting behind a pending motion frame. */
    Logger_Log(LOG_INFO, "[MODE] Sending stop before mode change");
    MotorTxDma_CancelPending();
    MotorDispatcher_SendRaw("stop");

    /* 2. Let the stop frame fully drain on every channel before issuing the
     *    mode command—otherwise the mode command could be staged behind the
     *    stop and arrive too early (or be dropped by the pending-slot policy). */
    if (!WaitForTxDrain(MODE_SWITCH_TX_DRAIN_MS))
    {
        Logger_Log(LOG_ERROR,
                   "[MODE] Stop TX did not drain within %ums; mode switch aborted, "
                   "local control mode unchanged (%s)",
                   (unsigned)MODE_SWITCH_TX_DRAIN_MS,
                   ControlMode_ToString(ControlMode_Get()));
        return;
    }

    /* 3. Dispatch the mode command to all motor controllers. */
    if (!MotorDispatcher_SendRaw(modeCmd))
    {
        Logger_Log(LOG_ERROR,
                   "[MODE] Failed to queue '%s' to one or more motors; "
                   "local control mode unchanged (%s)",
                   modeCmd,
                   ControlMode_ToString(ControlMode_Get()));
        return;
    }
    Logger_Log(LOG_INFO, "[MODE] Sent %s to all motors", modeCmd);

    /* 4. Only now advance the local control mode — after successful TX
     *    dispatch of the mode command.  Subsequent motion commands (e.g.
     *    f100 in RPM mode) will then be encoded as `rpm 100` / `rpm -100`. */
    ControlMode_Set(target);
    Logger_Log(LOG_INFO, "[MODE] Local control mode set to %s", modeName);
    Logger_Log(LOG_INFO,
               "[MODE] Mode command dispatched, not fully ACK-confirmed "
               "(F411 ACK parsing not wired for raw commands)");
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
    Logger_Log(LOG_INFO, "  While DISARM: only mode/status/help/stop/brake/tuning are accepted.");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "RPM mode commands:");
    Logger_Log(LOG_INFO, "  f0..f200         Forward RPM command");
    Logger_Log(LOG_INFO, "  b0..b200         Backward RPM command");
    Logger_Log(LOG_INFO, "  r0..r200         Right turn RPM command");
    Logger_Log(LOG_INFO, "  l0..l200         Left turn RPM command");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "PWM mode commands:");
    Logger_Log(LOG_INFO, "  fd0..fd4000      Forward PWM/duty command");
    Logger_Log(LOG_INFO, "  bd0..bd4000      Backward PWM/duty command");
    Logger_Log(LOG_INFO, "  rd0..rd4000      Right turn PWM/duty command");
    Logger_Log(LOG_INFO, "  ld0..ld4000      Left turn PWM/duty command");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Common commands:");
    Logger_Log(LOG_INFO, "  stop             Stop motors");
    Logger_Log(LOG_INFO, "  brake            Send brake command: x");
    Logger_Log(LOG_INFO, "  identify         Arm motors, then send identify to all motor UARTs");
    Logger_Log(LOG_INFO, "  status           Send status to all motor UARTs");
    Logger_Log(LOG_INFO, "  termstat         Terminal RX queue diagnostics");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Direct motor command:");
    Logger_Log(LOG_INFO, "  FL <text>        Send raw text only to Front Left motor");
    Logger_Log(LOG_INFO, "  FR <text>        Send raw text only to Front Right motor");
    Logger_Log(LOG_INFO, "  RL <text>        Send raw text only to Rear Left motor");
    Logger_Log(LOG_INFO, "  RR <text>        Send raw text only to Rear Right motor");
    Logger_Log(LOG_INFO, "Examples:");
    Logger_Log(LOG_INFO, "  FL status");
    Logger_Log(LOG_INFO, "  FR identify");
    Logger_Log(LOG_INFO, "  RL f100");
    Logger_Log(LOG_INFO, "  RR mode speed");
    Logger_Log(LOG_INFO, "");
    Logger_Log(LOG_INFO, "Motor tuning:");
    Logger_Log(LOG_INFO, "  FL base P1 P2 P3 P4 P5 P6 P7 P8");
    Logger_Log(LOG_INFO, "  FL boost P1 P2 P3 P4 P5 P6 P7 P8 MS");
    Logger_Log(LOG_INFO, "  FL kickduty VALUE    (or: FL kick duty VALUE)");
    Logger_Log(LOG_INFO, "  FL kickms VALUE      (or: FL kick ms VALUE)");
    Logger_Log(LOG_INFO, "  FL ramp UP DOWN");
    Logger_Log(LOG_INFO, "  FL pi KP KI");
    Logger_Log(LOG_INFO, "  FL telper MS");
    Logger_Log(LOG_INFO, "  ALL base / boost / kickduty / kickms / ramp / pi / telper");
    Logger_Log(LOG_INFO, "");
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
            case TCMD_TERMSTAT:     /* termstat (query) */
            case TCMD_MODE_QUERY:   /* mode (query) */
            case TCMD_STOP:         /* stop (safe) */
            case TCMD_BRAKE:        /* brake (safe) */
            case TCMD_MOTOR_TUNE:   /* tuning: no motion, allowed in DISARM */
                allowed = true;
                break;

            case TCMD_MOTOR_RAW:
                /* Empty payload (bare "FL") -> usage error, emit here so
                 * the handler's main switch does not need a DISARM copy. */
                if (cmd->rawPayload[0] == '\0')
                {
                    Logger_Log(LOG_ERROR,
                               "Usage: FL <text> | FR <text> | "
                               "RL <text> | RR <text>");
                    return;
                }
                /* Otherwise only safe payloads may pass; motion-causing
                 * raw commands (e.g. FL f100) are blocked. */
                allowed = IsSafeRawPayload(cmd->rawPayload);
                if (!allowed)
                {
                    Logger_Log(LOG_WARN,
                               "[DISARM] Direct motor command blocked");
                    return;
                }
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
            HandleControlModeSwitch(CONTROL_MODE_RPM);
            break;

        case TCMD_MODE_PWM:
            HandleControlModeSwitch(CONTROL_MODE_PWM);
            break;

        case TCMD_MODE_QUERY:
            Logger_Log(LOG_INFO, "Rover mode: %s",
                       OperatingMode_ToString(OperatingMode_Get()));
            break;

        case TCMD_IDENTIFY:
            /* Arm the motor controllers first (separate line), then
             * send the identify probe on its own line.  Waiting for the
             * TX DMA path to drain between the two guarantees ordering and
             * prevents the identify frame from being staged behind (or
             * dropped by) the pending-slot policy of the arm frame. */
            MotorDispatcher_SendRaw("arm service CURRENT_LIMITED_BENCH_SUPPLY");
            if (!WaitForTxDrain(MODE_SWITCH_TX_DRAIN_MS))
            {
                Logger_Log(LOG_ERROR,
                           "[IDENTIFY] Arm frame TX did not drain within %ums; "
                           "identify not sent",
                           (unsigned)MODE_SWITCH_TX_DRAIN_MS);
                break;
            }
            MotorDispatcher_SendRaw("identify");
            break;

        case TCMD_STATUS:
            MotorDispatcher_SendRaw("status");
            break;

        case TCMD_TERMSTAT:
            Logger_Log(LOG_INFO, "[TERM] rx_lines=%lu dropped=%lu pending=%u max_depth=%u",
                       (unsigned long)TerminalIf_GetReceivedLineCount(),
                       (unsigned long)TerminalIf_GetDroppedLineCount(),
                       (unsigned)TerminalIf_GetPendingLineCount(),
                       (unsigned)TerminalIf_GetMaxLineQueueDepth());
            break;

        case TCMD_MOTOR_RAW:
        {
            /* Bare motor tag with no payload -> usage error.
             * (DISARM-empty-payload path already returned earlier.) */
            if (cmd->rawPayload[0] == '\0')
            {
                Logger_Log(LOG_ERROR,
                           "Usage: FL <text> | FR <text> | "
                           "RL <text> | RR <text>");
                break;
            }

            Logger_Log(LOG_INFO, "[RAW][%s] %s",
                       MotorTagName(cmd->rawMotor), cmd->rawPayload);

            if (!MotorDispatcher_SendRawToMotor(cmd->rawMotor,
                                                cmd->rawPayload))
            {
                Logger_Log(LOG_ERROR, "Direct motor TX failed for %s",
                           MotorTagName(cmd->rawMotor));
            }
            break;
        }

        case TCMD_MOTOR_TUNE:
        {
            /* Validated tuning command — forward normalised payload. */
            if (!MotorDispatcher_SendTunePayload(cmd->tuneTarget,
                                                 cmd->tunePayload))
            {
                Logger_Log(LOG_ERROR, "[TUNE] Dispatch failed");
            }
            break;
        }

        default:
            break;
    }
}
